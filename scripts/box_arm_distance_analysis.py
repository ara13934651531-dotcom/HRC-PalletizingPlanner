#!/usr/bin/env python3
"""箱子-机械臂碰撞距离分析脚本
分析搬运段(seg3-5)中箱子角点到各碰撞体的最近距离
"""
import math
import numpy as np
import os

# S50 DH parameters (mm)
d_dh = [296.5, 336.2, 239.0, 158.5, 158.5, 134.5]
a_dh = [0, 900.0, 941.5, 0, 0, 0]
alpha_dh = [-math.pi/2, 0, 0, -math.pi/2, math.pi/2, 0]

# Collision geometry (mm) - capsule: {s, e, r}; ball: {s, e=None, r}
# These are defined in each link's LOCAL frame per CollisionGeometry.hpp
colliders = {
    'base':     {'s': np.array([0,0,20]),     'e': np.array([0,0,330]),    'r': 160, 'frame': 0},
    'lowerArm': {'s': np.array([0,0,340]),    'e': np.array([900,0,340]),  'r': 140, 'frame': 1},
    'elbow':    {'s': np.array([-10,0,60]),   'e': np.array([941.5,0,60]), 'r': 120, 'frame': 2},
    'upperArm': {'s': np.array([0,0,-50]),    'e': np.array([0,0,100]),    'r': 100, 'frame': 4},
    'wrist':    {'s': np.array([0,0,20]),     'e': None,                   'r': 140, 'frame': 5},
}

box_lx, box_wy, box_hz = 350, 280, 250  # mm

def dh_transform(theta, d_val, a_val, alpha_val):
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha_val), math.sin(alpha_val)
    return np.array([
        [ct, -st*ca,  st*sa, a_val*ct],
        [st,  ct*ca, -ct*sa, a_val*st],
        [0,   sa,     ca,    d_val],
        [0,   0,      0,     1]
    ])

def fk_all_frames(q_deg):
    q = [math.radians(x) for x in q_deg]
    frames = [np.eye(4)]
    T = np.eye(4)
    for i in range(6):
        Ti = dh_transform(q[i], d_dh[i], a_dh[i], alpha_dh[i])
        T = T @ Ti
        frames.append(T.copy())
    return frames

def point_to_segment_dist(p, seg_a, seg_b):
    ab = seg_b - seg_a
    ap = p - seg_a
    ab2 = np.dot(ab, ab)
    if ab2 < 1e-12:
        return np.linalg.norm(ap)
    t = max(0, min(1, np.dot(ap, ab) / ab2))
    closest = seg_a + t * ab
    return np.linalg.norm(p - closest)

def box_corners_world(T6):
    """Generate 8 box corners in world frame.
    TCP朝下: 法兰Z+方向=TCP方向=朝下
    箱子吸盘在箱顶, 箱体从TCP向下延伸box_hz
    法兰坐标系: TCP在(0,0,d6), 箱顶=TCP, 箱底=(0,0,d6+box_hz)
    """
    corners = []
    for sx in [-1, 1]:
        for sy in [-1, 1]:
            for sz_frac in [0, 1]:
                local = np.array([sx*box_lx/2, sy*box_wy/2, d_dh[5] + sz_frac*box_hz])
                world = T6[:3,:3] @ local + T6[:3,3]
                corners.append(world)
    return corners

def analyze_pose(q_deg, label=""):
    frames = fk_all_frames(q_deg)
    T6 = frames[6]
    tcp_pos = T6[:3, 3]
    z_axis = T6[:3, 2]
    
    corners = box_corners_world(T6)
    
    print(f"\n{'='*60}")
    print(f"姿态: {label}")
    print(f"  q = [{', '.join(f'{x:.2f}' for x in q_deg)}] deg")
    print(f"  TCP = ({tcp_pos[0]:.1f}, {tcp_pos[1]:.1f}, {tcp_pos[2]:.1f}) mm")
    print(f"  Z轴 = ({z_axis[0]:.4f}, {z_axis[1]:.4f}, {z_axis[2]:.4f})")
    
    results = {}
    for name, coll in colliders.items():
        T = frames[coll['frame']]
        s_w = T[:3,:3] @ coll['s'] + T[:3,3]
        e_w = T[:3,:3] @ coll['e'] + T[:3,3] if coll['e'] is not None else None
        
        min_d = float('inf')
        for c in corners:
            if e_w is not None:
                dd = point_to_segment_dist(c, s_w, e_w)
            else:
                dd = np.linalg.norm(c - s_w)
            min_d = min(min_d, dd)
        
        surf = min_d - coll['r']
        marker = ""
        if surf < 0: marker = " *** 碰撞! ***"
        elif surf < 50: marker = " !! 危险!"
        elif surf < 100: marker = " ! 注意"
        
        print(f"  {name:10s}: 中心线距={min_d:7.1f}mm  表面距={surf:7.1f}mm{marker}")
        results[name] = surf
    
    return results

def load_trajectory(filepath):
    """Load trajectory file, return rows as list of lists"""
    rows = []
    with open(filepath) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            vals = line.split()
            if len(vals) >= 19:
                rows.append([float(v) for v in vals])
    return rows

def main():
    print("="*60)
    print("箱子(350x280x250mm) vs 机械臂碰撞体 距离分析")
    print("="*60)
    
    # 1. 分析几个关键姿态
    print("\n>>> 1. 关键姿态分析")
    
    # seg4 start: 传送带出发方向
    analyze_pose([154.59, -85.39, 118.35, -58.34, 152.26, 157.22],
                 "seg4起点 (传送带上方, 高位)")
    
    # seg4 end: 框架方向
    analyze_pose([-110.80, -52.70, 125.00, 17.70, 270.00, -159.20],
                 "seg4终点 (框架上方)")
    
    # HOME with TCP horizontal
    analyze_pose([0, -90, 0, 0, 90, 0], "HOME (参考)")
    
    # 2. 扫描完整seg3-5轨迹
    print("\n\n>>> 2. 完整搬运段(seg3-5)轨迹扫描")
    
    traj_path = os.path.join(os.path.dirname(__file__), '..', 'data', 'so_palletizing_trajectory.txt')
    if not os.path.exists(traj_path):
        traj_path = '/home/ara/文档/X86_test/data/so_palletizing_trajectory.txt'
    
    if os.path.exists(traj_path):
        rows = load_trajectory(traj_path)
        carrying_rows = [r for r in rows if 3 <= r[1] <= 5]
        
        print(f"  搬运段帧数: {len(carrying_rows)}")
        
        global_min = {}
        for name in colliders:
            global_min[name] = float('inf')
        
        worst_frame = None
        worst_dist = float('inf')
        
        for i, row in enumerate(carrying_rows):
            seg = int(row[1])
            q_deg = row[3:9]  # columns 4-9 are q1..q6 in deg
            
            frames = fk_all_frames(q_deg)
            T6 = frames[6]
            corners = box_corners_world(T6)
            
            for name, coll in colliders.items():
                T = frames[coll['frame']]
                s_w = T[:3,:3] @ coll['s'] + T[:3,3]
                e_w = T[:3,:3] @ coll['e'] + T[:3,3] if coll['e'] is not None else None
                
                min_d = float('inf')
                for c in corners:
                    if e_w is not None:
                        dd = point_to_segment_dist(c, s_w, e_w)
                    else:
                        dd = np.linalg.norm(c - s_w)
                    min_d = min(min_d, dd)
                
                surf = min_d - coll['r']
                if surf < global_min[name]:
                    global_min[name] = surf
                
                if surf < worst_dist:
                    worst_dist = surf
                    worst_frame = (i, seg, q_deg, name, surf)
        
        print(f"\n  各碰撞体 vs 箱子角点 最小表面距离:")
        print(f"  {'碰撞体':12s}  {'最小表面距(mm)':>15s}  {'状态'}")
        print(f"  {'-'*50}")
        for name in colliders:
            d = global_min[name]
            marker = ""
            if d < 0: marker = "碰撞!"
            elif d < 50: marker = "危险"
            elif d < 100: marker = "注意"
            else: marker = "安全"
            print(f"  {name:12s}  {d:15.1f}  {marker}")
        
        if worst_frame:
            i, seg, q, name, d = worst_frame
            print(f"\n  最危险帧: frame={i}, seg={seg}, collider={name}, 距离={d:.1f}mm")
            print(f"  关节角: [{', '.join(f'{x:.2f}' for x in q)}] deg")
            
            # Detail analysis of worst frame
            analyze_pose(q, f"最危险帧 (frame={i}, seg={seg})")
    else:
        print(f"  未找到轨迹文件: {traj_path}")
    
    # 3. Summary
    print("\n\n>>> 3. 结论")
    print("="*60)
    print("SO库工具球 (z=-400, r=120) 分析:")
    print(f"  工具球Z覆盖: [-520, -280]mm (法兰坐标系)")
    print(f"  箱子Z覆盖:   [+134.5, +384.5]mm (法兰坐标系)")
    print(f"  间距: 414.5mm — 完全没有重叠!")
    print(f"  51.9mm = 工具球(link6) vs 腕部球(link4) 的表面距离")
    print(f"  与箱子是否碰撞机械臂无关!")
    print()
    print("箱子实际碰撞分析:")
    if any(v < 0 for v in global_min.values()):
        print("  !!! 存在碰撞 !!! — 箱子角点与机械臂碰撞体重叠")
    elif any(v < 50 for v in global_min.values()):
        print("  !!! 有危险接近 !!! — 表面距离<50mm")
    else:
        min_overall = min(global_min.values())
        print(f"  最小表面距离: {min_overall:.1f}mm")
        if min_overall > 100:
            print("  箱子角点与机械臂本体距离尚可(>100mm)")
        else:
            print("  箱子角点与机械臂接近, 需要注意")
    print("="*60)

if __name__ == '__main__':
    main()
