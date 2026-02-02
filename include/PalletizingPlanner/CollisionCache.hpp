/**
 * @file CollisionCache.hpp
 * @brief 碰撞检测缓存 - 避免重复碰撞检测
 * 
 * 特性：
 * - 空间哈希缓存
 * - 可调分辨率
 * - LRU淘汰策略
 * - 线程安全选项
 * 
 * @author GitHub Copilot
 * @version 1.0.0
 * @date 2026-01-29
 */

#pragma once

#include "Types.hpp"
#include <unordered_map>
#include <list>
#include <mutex>
#include <functional>

namespace palletizing {

/**
 * @brief 碰撞检测缓存键
 */
struct CollisionCacheKey {
    std::array<int64_t, 6> discreteQ;  // 离散化的关节角度
    
    bool operator==(const CollisionCacheKey& other) const {
        return discreteQ == other.discreteQ;
    }
};

/**
 * @brief 缓存键哈希函数
 */
struct CollisionCacheKeyHash {
    size_t operator()(const CollisionCacheKey& key) const {
        size_t hash = 0;
        for (int i = 0; i < 6; ++i) {
            // FNV-1a 哈希
            hash ^= static_cast<size_t>(key.discreteQ[i]);
            hash *= 0x100000001b3ULL;
        }
        return hash;
    }
};

/**
 * @brief 碰撞检测缓存配置
 */
struct CollisionCacheConfig {
    double resolution = 0.5;        // 离散化分辨率 (度)
    size_t maxCacheSize = 100000;   // 最大缓存条目数
    bool threadSafe = false;        // 是否需要线程安全
    
    static CollisionCacheConfig defaultConfig() {
        return CollisionCacheConfig();
    }
    
    static CollisionCacheConfig highResolution() {
        CollisionCacheConfig config;
        config.resolution = 0.1;
        config.maxCacheSize = 500000;
        return config;
    }
    
    static CollisionCacheConfig lowMemory() {
        CollisionCacheConfig config;
        config.resolution = 1.0;
        config.maxCacheSize = 10000;
        return config;
    }
};

/**
 * @brief 碰撞检测缓存（LRU策略）
 */
class CollisionCache {
public:
    struct CacheStats {
        size_t hits = 0;
        size_t misses = 0;
        size_t evictions = 0;
        size_t currentSize = 0;
        
        double hitRate() const {
            size_t total = hits + misses;
            return total > 0 ? static_cast<double>(hits) / total : 0.0;
        }
    };
    
    explicit CollisionCache(const CollisionCacheConfig& config = CollisionCacheConfig::defaultConfig())
        : config_(config) {}
    
    /**
     * @brief 查询缓存
     * @param q 关节配置
     * @param result 如果命中，输出结果
     * @return 是否命中
     */
    bool lookup(const JointConfig& q, bool& result) {
        CollisionCacheKey key = makeKey(q);
        
        if (config_.threadSafe) {
            std::lock_guard<std::mutex> lock(mutex_);
            return lookupInternal(key, result);
        } else {
            return lookupInternal(key, result);
        }
    }
    
    /**
     * @brief 插入缓存
     * @param q 关节配置
     * @param collisionFree 是否无碰撞
     */
    void insert(const JointConfig& q, bool collisionFree) {
        CollisionCacheKey key = makeKey(q);
        
        if (config_.threadSafe) {
            std::lock_guard<std::mutex> lock(mutex_);
            insertInternal(key, collisionFree);
        } else {
            insertInternal(key, collisionFree);
        }
    }
    
    /**
     * @brief 批量查询（优化内存访问模式）
     * @param configs 关节配置列表
     * @param results 输出结果（-1表示未命中，0表示碰撞，1表示无碰撞）
     */
    void batchLookup(const std::vector<JointConfig>& configs, std::vector<int8_t>& results) {
        results.resize(configs.size());
        
        if (config_.threadSafe) {
            std::lock_guard<std::mutex> lock(mutex_);
            for (size_t i = 0; i < configs.size(); ++i) {
                bool result;
                if (lookupInternal(makeKey(configs[i]), result)) {
                    results[i] = result ? 1 : 0;
                } else {
                    results[i] = -1;
                }
            }
        } else {
            for (size_t i = 0; i < configs.size(); ++i) {
                bool result;
                if (lookupInternal(makeKey(configs[i]), result)) {
                    results[i] = result ? 1 : 0;
                } else {
                    results[i] = -1;
                }
            }
        }
    }
    
    /**
     * @brief 批量插入
     */
    void batchInsert(const std::vector<JointConfig>& configs, const std::vector<bool>& results) {
        if (configs.size() != results.size()) return;
        
        if (config_.threadSafe) {
            std::lock_guard<std::mutex> lock(mutex_);
            for (size_t i = 0; i < configs.size(); ++i) {
                insertInternal(makeKey(configs[i]), results[i]);
            }
        } else {
            for (size_t i = 0; i < configs.size(); ++i) {
                insertInternal(makeKey(configs[i]), results[i]);
            }
        }
    }
    
    /**
     * @brief 获取统计信息
     */
    CacheStats getStats() const {
        if (config_.threadSafe) {
            std::lock_guard<std::mutex> lock(mutex_);
            return stats_;
        }
        return stats_;
    }
    
    /**
     * @brief 清空缓存
     */
    void clear() {
        if (config_.threadSafe) {
            std::lock_guard<std::mutex> lock(mutex_);
        }
        cache_.clear();
        lruList_.clear();
        stats_ = CacheStats();
    }
    
    /**
     * @brief 预热缓存
     */
    template<typename CollisionChecker>
    void warmup(CollisionChecker& checker, const std::vector<JointConfig>& samples) {
        for (const auto& q : samples) {
            bool result;
            if (!lookup(q, result)) {
                bool collisionFree = checker.isCollisionFree(q);
                insert(q, collisionFree);
            }
        }
    }
    
private:
    using LRUList = std::list<CollisionCacheKey>;
    using CacheMap = std::unordered_map<CollisionCacheKey, 
                                         std::pair<bool, typename LRUList::iterator>,
                                         CollisionCacheKeyHash>;
    
    CollisionCacheKey makeKey(const JointConfig& q) const {
        CollisionCacheKey key;
        double invRes = 1.0 / config_.resolution;
        for (int i = 0; i < 6; ++i) {
            // 离散化到分辨率网格
            key.discreteQ[i] = static_cast<int64_t>(std::floor(q.q[i] * invRes + 0.5));
        }
        return key;
    }
    
    bool lookupInternal(const CollisionCacheKey& key, bool& result) {
        auto it = cache_.find(key);
        if (it == cache_.end()) {
            stats_.misses++;
            return false;
        }
        
        // 更新LRU位置
        lruList_.splice(lruList_.begin(), lruList_, it->second.second);
        
        result = it->second.first;
        stats_.hits++;
        return true;
    }
    
    void insertInternal(const CollisionCacheKey& key, bool collisionFree) {
        auto it = cache_.find(key);
        if (it != cache_.end()) {
            // 已存在，更新值和LRU位置
            it->second.first = collisionFree;
            lruList_.splice(lruList_.begin(), lruList_, it->second.second);
            return;
        }
        
        // 检查是否需要淘汰
        while (cache_.size() >= config_.maxCacheSize) {
            // 淘汰最久未使用的
            auto& oldest = lruList_.back();
            cache_.erase(oldest);
            lruList_.pop_back();
            stats_.evictions++;
        }
        
        // 插入新条目
        lruList_.push_front(key);
        cache_[key] = {collisionFree, lruList_.begin()};
        stats_.currentSize = cache_.size();
    }
    
    CollisionCacheConfig config_;
    CacheMap cache_;
    LRUList lruList_;
    CacheStats stats_;
    mutable std::mutex mutex_;
};

/**
 * @brief 带缓存的碰撞检测器包装
 */
template<typename BaseChecker>
class CachedCollisionChecker {
public:
    CachedCollisionChecker(BaseChecker& checker, 
                           const CollisionCacheConfig& cacheConfig = CollisionCacheConfig::defaultConfig())
        : checker_(checker), cache_(cacheConfig) {}
    
    bool isCollisionFree(const JointConfig& q) {
        bool result;
        if (cache_.lookup(q, result)) {
            return result;
        }
        
        result = checker_.isCollisionFree(q);
        cache_.insert(q, result);
        return result;
    }
    
    bool isPathCollisionFree(const JointConfig& start, const JointConfig& end, double resolution = 0.5) {
        // 路径检测不使用缓存，因为中间点会变化
        // 但端点可以使用缓存
        if (!isCollisionFree(start) || !isCollisionFree(end)) {
            return false;
        }
        
        // 中间点直接检测
        double dist = start.distanceTo(end);
        int numSteps = std::max(2, static_cast<int>(std::ceil(dist / resolution)));
        
        for (int i = 1; i < numSteps; ++i) {
            double t = static_cast<double>(i) / numSteps;
            JointConfig mid = start.interpolate(end, t);
            if (!isCollisionFree(mid)) {
                return false;
            }
        }
        
        return true;
    }
    
    CollisionCache::CacheStats getCacheStats() const {
        return cache_.getStats();
    }
    
    void clearCache() {
        cache_.clear();
    }
    
    BaseChecker& baseChecker() { return checker_; }
    
private:
    BaseChecker& checker_;
    CollisionCache cache_;
};

} // namespace palletizing
