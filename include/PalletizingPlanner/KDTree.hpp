/**
 * @file KDTree.hpp
 * @brief 高性能KD-Tree实现 - 用于加速最近邻搜索
 * 
 * 特性：
 * - O(log n) 最近邻查询
 * - O(log n) 范围查询
 * - 支持6维关节空间
 * - 缓存友好的内存布局
 * 
 * @author GitHub Copilot
 * @version 1.0.0
 * @date 2026-01-29
 */

#pragma once

#include "Types.hpp"
#include <algorithm>
#include <queue>
#include <stack>
#include <limits>

namespace palletizing {

/**
 * @brief 高性能6D KD-Tree
 */
class KDTree6D {
public:
    struct Node {
        int pointIndex;      // 点索引
        int splitDim;        // 分割维度
        double splitValue;   // 分割值
        int left = -1;       // 左子节点索引
        int right = -1;      // 右子节点索引
    };
    
    KDTree6D() = default;
    
    /**
     * @brief 从点集构建KD-Tree
     * @param points 点集
     */
    void build(const std::vector<JointConfig>& points) {
        points_ = &points;
        nodes_.clear();
        nodes_.reserve(points.size());
        
        if (points.empty()) return;
        
        // 创建索引数组
        std::vector<int> indices(points.size());
        std::iota(indices.begin(), indices.end(), 0);
        
        // 递归构建
        rootIndex_ = buildRecursive(indices, 0, static_cast<int>(indices.size()), 0);
    }
    
    /**
     * @brief 查找最近邻
     * @param query 查询点
     * @return 最近邻的索引
     */
    int findNearest(const JointConfig& query) const {
        if (nodes_.empty()) return -1;
        
        int bestIndex = -1;
        double bestDist = std::numeric_limits<double>::infinity();
        
        findNearestRecursive(rootIndex_, query, bestIndex, bestDist);
        
        return bestIndex;
    }
    
    /**
     * @brief 查找给定半径内的所有点
     * @param query 查询点
     * @param radius 搜索半径
     * @return 点索引列表
     */
    std::vector<int> findWithinRadius(const JointConfig& query, double radius) const {
        std::vector<int> result;
        if (nodes_.empty()) return result;
        
        double radiusSq = radius * radius;
        findWithinRadiusRecursive(rootIndex_, query, radiusSq, result);
        
        return result;
    }
    
    /**
     * @brief 查找K个最近邻
     * @param query 查询点
     * @param k 邻居数量
     * @return 点索引列表（按距离排序）
     */
    std::vector<int> findKNearest(const JointConfig& query, int k) const {
        if (nodes_.empty() || k <= 0) return {};
        
        // 使用最大堆存储k个最近邻
        std::priority_queue<std::pair<double, int>> maxHeap;
        
        findKNearestRecursive(rootIndex_, query, k, maxHeap);
        
        // 提取结果
        std::vector<int> result;
        result.reserve(maxHeap.size());
        while (!maxHeap.empty()) {
            result.push_back(maxHeap.top().second);
            maxHeap.pop();
        }
        std::reverse(result.begin(), result.end());
        
        return result;
    }
    
    /**
     * @brief 增量添加点
     */
    void addPoint(int pointIndex) {
        if (nodes_.empty()) {
            Node node;
            node.pointIndex = pointIndex;
            node.splitDim = 0;
            node.splitValue = (*points_)[pointIndex].q[0];
            nodes_.push_back(node);
            rootIndex_ = 0;
            return;
        }
        
        // 找到插入位置
        int current = rootIndex_;
        while (true) {
            const Node& node = nodes_[current];
            double value = (*points_)[pointIndex].q[node.splitDim];
            
            if (value < node.splitValue) {
                if (node.left < 0) {
                    Node newNode;
                    newNode.pointIndex = pointIndex;
                    newNode.splitDim = (node.splitDim + 1) % 6;
                    newNode.splitValue = (*points_)[pointIndex].q[newNode.splitDim];
                    
                    nodes_[current].left = static_cast<int>(nodes_.size());
                    nodes_.push_back(newNode);
                    break;
                }
                current = node.left;
            } else {
                if (node.right < 0) {
                    Node newNode;
                    newNode.pointIndex = pointIndex;
                    newNode.splitDim = (node.splitDim + 1) % 6;
                    newNode.splitValue = (*points_)[pointIndex].q[newNode.splitDim];
                    
                    nodes_[current].right = static_cast<int>(nodes_.size());
                    nodes_.push_back(newNode);
                    break;
                }
                current = node.right;
            }
        }
    }
    
    size_t size() const { return nodes_.size(); }
    bool empty() const { return nodes_.empty(); }
    
private:
    int buildRecursive(std::vector<int>& indices, int start, int end, int depth) {
        if (start >= end) return -1;
        
        int splitDim = depth % 6;
        
        // 找中位数
        int mid = (start + end) / 2;
        std::nth_element(indices.begin() + start, indices.begin() + mid, indices.begin() + end,
            [this, splitDim](int a, int b) {
                return (*points_)[a].q[splitDim] < (*points_)[b].q[splitDim];
            });
        
        // 创建节点
        Node node;
        node.pointIndex = indices[mid];
        node.splitDim = splitDim;
        node.splitValue = (*points_)[indices[mid]].q[splitDim];
        
        int nodeIndex = static_cast<int>(nodes_.size());
        nodes_.push_back(node);
        
        // 递归构建子树
        nodes_[nodeIndex].left = buildRecursive(indices, start, mid, depth + 1);
        nodes_[nodeIndex].right = buildRecursive(indices, mid + 1, end, depth + 1);
        
        return nodeIndex;
    }
    
    void findNearestRecursive(int nodeIndex, const JointConfig& query,
                               int& bestIndex, double& bestDist) const {
        if (nodeIndex < 0) return;
        
        const Node& node = nodes_[nodeIndex];
        
        // 计算到当前点的距离
        double dist = (*points_)[node.pointIndex].distanceTo(query);
        if (dist < bestDist) {
            bestDist = dist;
            bestIndex = node.pointIndex;
        }
        
        // 决定先搜索哪个子树
        double diff = query.q[node.splitDim] - node.splitValue;
        int first = diff < 0 ? node.left : node.right;
        int second = diff < 0 ? node.right : node.left;
        
        // 搜索较近的子树
        findNearestRecursive(first, query, bestIndex, bestDist);
        
        // 如果分割平面可能有更近的点，搜索另一个子树
        if (std::abs(diff) < bestDist) {
            findNearestRecursive(second, query, bestIndex, bestDist);
        }
    }
    
    void findWithinRadiusRecursive(int nodeIndex, const JointConfig& query,
                                    double radiusSq, std::vector<int>& result) const {
        if (nodeIndex < 0) return;
        
        const Node& node = nodes_[nodeIndex];
        
        // 检查当前点
        double distSq = 0;
        for (int i = 0; i < 6; ++i) {
            double d = (*points_)[node.pointIndex].q[i] - query.q[i];
            distSq += d * d;
        }
        
        if (distSq <= radiusSq) {
            result.push_back(node.pointIndex);
        }
        
        // 决定搜索哪些子树
        double diff = query.q[node.splitDim] - node.splitValue;
        
        if (diff < 0) {
            findWithinRadiusRecursive(node.left, query, radiusSq, result);
            if (diff * diff <= radiusSq) {
                findWithinRadiusRecursive(node.right, query, radiusSq, result);
            }
        } else {
            findWithinRadiusRecursive(node.right, query, radiusSq, result);
            if (diff * diff <= radiusSq) {
                findWithinRadiusRecursive(node.left, query, radiusSq, result);
            }
        }
    }
    
    void findKNearestRecursive(int nodeIndex, const JointConfig& query, int k,
                                std::priority_queue<std::pair<double, int>>& maxHeap) const {
        if (nodeIndex < 0) return;
        
        const Node& node = nodes_[nodeIndex];
        
        // 计算距离
        double dist = (*points_)[node.pointIndex].distanceTo(query);
        
        if (static_cast<int>(maxHeap.size()) < k) {
            maxHeap.emplace(dist, node.pointIndex);
        } else if (dist < maxHeap.top().first) {
            maxHeap.pop();
            maxHeap.emplace(dist, node.pointIndex);
        }
        
        // 搜索子树
        double diff = query.q[node.splitDim] - node.splitValue;
        int first = diff < 0 ? node.left : node.right;
        int second = diff < 0 ? node.right : node.left;
        
        findKNearestRecursive(first, query, k, maxHeap);
        
        // 检查是否需要搜索另一个子树
        double maxDist = maxHeap.empty() ? std::numeric_limits<double>::infinity() : maxHeap.top().first;
        if (static_cast<int>(maxHeap.size()) < k || std::abs(diff) < maxDist) {
            findKNearestRecursive(second, query, k, maxHeap);
        }
    }
    
    const std::vector<JointConfig>* points_ = nullptr;
    std::vector<Node> nodes_;
    int rootIndex_ = -1;
};

} // namespace palletizing
