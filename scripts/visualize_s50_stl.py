#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
HR_S50-2000 STL 3D 仿真可视化 — 碰撞检测 + 码垛轨迹分析

使用 URDF/STL 网格模型进行高精度 3D 可视化:
  1. 加载 S50_ros2 中的 STL 文件, 显示真实机器人外观
  2. 从 data/rml_collision_profile.csv 读取碰撞仿真数据
  3. 从 data/rml_palletizing_profile.csv 读取码垛仿真数据
  4. 生成多张分析图表:
     - 机器人3D姿态 (STL网格)
     - 碰撞距离时间曲线
     - S曲线速度/加速度剖面
     - TCP 3D轨迹
     - 码垛节拍统计

Copyright (c) 2026 Guangdong Huayan Robotics Co., Ltd.
Date: 2026-02-23
"""
import os
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')  # 无显示后端
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import struct

# ============================================================================
# HR_S50-2000 DH 参数 (mm → m for calculation, mm for display)
# ============================================================================
DH = {
    'd1': 0.2965, 'd2': 0.3362, 'd3': 0.239,
    'd4': 0.1585, 'd5': 0.1585, 'd6': 0.1345,
    'a2': 0.900, 'a3': 0.9415
}

# URDF 中的关节原点 (从 S50.urdf.xacro 提取)
# joint: [xyz, rpy]
URDF_JOINTS = [
    # base → link1: xyz="0 0 0.2833" rpy="0 0 1.5708"
    {'xyz': [0, 0, 0.2833], 'rpy': [0, 0, 1.5708]},
    # link1 → link2: xyz="-0.3345 0 0" rpy="1.5708 0 -1.5708"
    {'xyz': [-0.3345, 0, 0], 'rpy': [1.5708, 0, -1.5708]},
    # link2 → link3: xyz="-0.9 0 -0.239" rpy="0 0 3.1416"
    {'xyz': [-0.9, 0, -0.239], 'rpy': [0, 0, 3.1416]},
    # link3 → link4: xyz="0.9415 0 0" rpy="0 0 0"
    {'xyz': [0.9415, 0, 0], 'rpy': [0, 0, 0]},
    # link4 → link5: xyz="0 0 0.1585" rpy="-1.5708 0 0"
    {'xyz': [0, 0, 0.1585], 'rpy': [-1.5708, 0, 0]},
    # link5 → link6: xyz="0 0 0.1585" rpy="1.5708 0 0"
    {'xyz': [0, 0, 0.1585], 'rpy': [1.5708, 0, 0]},
]

# 末端关节 end_link: xyz="0 0 0.1345" rpy="0 0 3.1416"
URDF_END = {'xyz': [0, 0, 0.1345], 'rpy': [0, 0, 3.1416]}

# ============================================================================
# STL 文件加载
# ============================================================================
def load_stl_binary(filepath):
    """加载二进制STL文件, 返回三角面片列表 (N×3×3)"""
    with open(filepath, 'rb') as f:
        header = f.read(80)
        num_triangles = struct.unpack('<I', f.read(4))[0]
        triangles = []
        for _ in range(num_triangles):
            normal = struct.unpack('<3f', f.read(12))
            v1 = struct.unpack('<3f', f.read(12))
            v2 = struct.unpack('<3f', f.read(12))
            v3 = struct.unpack('<3f', f.read(12))
            attr = struct.unpack('<H', f.read(2))
            triangles.append([v1, v2, v3])
        return np.array(triangles)  # (N, 3, 3)

def load_stl_auto(filepath):
    """自动检测STL格式 (ASCII/Binary) 并加载"""
    with open(filepath, 'rb') as f:
        header = f.read(80)
    if b'solid' in header[:5]:
        # 可能是ASCII, 但也可能是binary起始恰好含solid
        try:
            return load_stl_binary(filepath)
        except:
            pass
    return load_stl_binary(filepath)

# ============================================================================
# 旋转矩阵
# ============================================================================
def rot_x(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1,0,0],[0,c,-s],[0,s,c]])

def rot_y(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c,0,s],[0,1,0],[-s,0,c]])

def rot_z(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c,-s,0],[s,c,0],[0,0,1]])

def rpy_to_matrix(rpy):
    """RPY (roll, pitch, yaw) → 旋转矩阵"""
    return rot_z(rpy[2]) @ rot_y(rpy[1]) @ rot_x(rpy[0])

def make_transform(xyz, rpy):
    """生成 4×4 齐次变换矩阵"""
    T = np.eye(4)
    T[:3,:3] = rpy_to_matrix(rpy)
    T[:3, 3] = xyz
    return T

# ============================================================================
# 正运动学 (URDF 风格)
# ============================================================================
def fk_urdf(q_rad):
    """
    使用 URDF 关节定义计算各连杆位姿
    q_rad: 6个关节角 (rad)
    返回: 7个 4×4 矩阵 [T_base, T_link1, ..., T_link6]
    """
    transforms = [np.eye(4)]  # base
    
    T_current = np.eye(4)
    for i, joint in enumerate(URDF_JOINTS):
        # 关节原点变换 (parent → child)
        T_joint = make_transform(joint['xyz'], joint['rpy'])
        # 关节旋转
        T_rot = np.eye(4)
        T_rot[:3,:3] = rot_z(q_rad[i])
        T_current = T_current @ T_joint @ T_rot
        transforms.append(T_current.copy())
    
    return transforms

# ============================================================================
# 标准 DH 正运动学 (与C++一致)
# ============================================================================
def fk_dh(q_rad):
    """DH正运动学, 返回7个连杆变换矩阵"""
    d1, d2, d3, d4, d5, d6 = DH['d1'], DH['d2'], DH['d3'], DH['d4'], DH['d5'], DH['d6']
    a2, a3 = DH['a2'], DH['a3']
    
    dh_params = [
        (q_rad[0], d1,     0,   np.pi/2),
        (q_rad[1], 0,     -a2,  0),
        (q_rad[2], 0,     -a3,  0),
        (q_rad[3], d2-d3+d4, 0, np.pi/2),
        (q_rad[4], d5,    0,   -np.pi/2),
        (q_rad[5], d6,    0,    0),
    ]
    
    transforms = [np.eye(4)]
    T = np.eye(4)
    for theta, d, a, alpha in dh_params:
        ct, st = np.cos(theta), np.sin(theta)
        ca, sa = np.cos(alpha), np.sin(alpha)
        Ti = np.array([
            [ct, -st*ca,  st*sa, a*ct],
            [st,  ct*ca, -ct*sa, a*st],
            [0,   sa,     ca,    d],
            [0,   0,      0,     1]
        ])
        T = T @ Ti
        transforms.append(T.copy())
    return transforms

# ============================================================================
# 3D 可视化: STL 网格机器人
# ============================================================================
def plot_robot_stl(ax, q_deg, stl_dir, alpha=0.7, color_safe=(0.8, 0.85, 0.92),
                   color_collision=(0.9, 0.3, 0.3)):
    """在 3D 坐标轴上绘制 STL 网格机器人"""
    q_rad = np.deg2rad(q_deg)
    transforms = fk_dh(q_rad)
    
    stl_files = [
        'elfin_base.STL', 'elfin_link1.STL', 'elfin_link2.STL',
        'elfin_link3.STL', 'elfin_link4.STL', 'elfin_link5.STL',
        'elfin_link6.STL'
    ]
    
    link_colors = [
        (0.60, 0.62, 0.66),  # base - 深灰
        (0.85, 0.87, 0.92),  # link1
        (0.85, 0.87, 0.92),  # link2
        (0.85, 0.87, 0.92),  # link3
        (0.85, 0.87, 0.92),  # link4
        (0.85, 0.87, 0.92),  # link5
        (0.75, 0.77, 0.82),  # link6 - 末端
    ]
    
    for i, stl_file in enumerate(stl_files):
        fpath = os.path.join(stl_dir, stl_file)
        if not os.path.exists(fpath):
            continue
        
        try:
            tris = load_stl_auto(fpath)
        except Exception as e:
            print(f"  [警告] 无法加载 {stl_file}: {e}")
            continue
        
        if i < len(transforms):
            T = transforms[i]
            # STL 顶点从 mm → m, 然后变换
            verts = tris.reshape(-1, 3) / 1000.0  # STL单位mm → 转m
            ones = np.ones((verts.shape[0], 1))
            verts_h = np.hstack([verts, ones])
            verts_t = (T @ verts_h.T).T[:, :3]
            tris_t = verts_t.reshape(-1, 3, 3)
            
            # 只绘制部分面片 (降采样提速)
            step = max(1, len(tris_t) // 800)
            mesh = Poly3DCollection(tris_t[::step] * 1000,  # 显示mm
                                    alpha=alpha, linewidth=0.1,
                                    edgecolor=(0.5, 0.5, 0.5, 0.2))
            mesh.set_facecolor(link_colors[i] if i < len(link_colors) else color_safe)
            ax.add_collection3d(mesh)
    
    # 绘制关节位置连线 (骨架)
    positions = [T[:3, 3] * 1000 for T in transforms]  # mm
    xs = [p[0] for p in positions]
    ys = [p[1] for p in positions]
    zs = [p[2] for p in positions]
    ax.plot(xs, ys, zs, 'o-', color='#333333', linewidth=2.5, markersize=5, zorder=10)
    
    # TCP 标记
    ax.scatter([xs[-1]], [ys[-1]], [zs[-1]], c='red', s=80, marker='*', zorder=11)
    
    return positions

# ============================================================================
# 数据分析图表
# ============================================================================
def analyze_collision_data(data_dir, output_dir):
    """分析碰撞仿真数据并生成图表"""
    csv_path = os.path.join(data_dir, 'rml_collision_profile.csv')
    if not os.path.exists(csv_path):
        print(f"  [跳过] {csv_path} 不存在")
        return
    
    print(f"  加载碰撞数据: {csv_path}")
    data = np.genfromtxt(csv_path, delimiter=',', skip_header=1, dtype=float)
    if data.ndim < 2 or data.shape[0] < 2:
        print("  [跳过] 数据不足")
        return
    
    # CSV格式: scenario,time_s,q1_deg,...,q6_deg,v_peak_dps,selfMinDist_mm,selfCollision,tcpX_mm,tcpY_mm,tcpZ_mm
    scenario = data[:, 0].astype(int)
    time = data[:, 1]
    dist = data[:, 9]     # selfMinDist_mm (已是mm)
    coll = data[:, 10]    # selfCollision
    tcp_x = data[:, 11]   # tcpX_mm
    tcp_y = data[:, 12]   # tcpY_mm
    tcp_z = data[:, 13]   # tcpZ_mm
    
    # 图1: 碰撞距离时间曲线
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    
    # 碰撞距离
    ax1 = axes[0]
    safe_mask = coll == 0
    ax1.plot(time[safe_mask], dist[safe_mask], '.', color='#2196F3',
             markersize=1, alpha=0.6, label='Safe')
    danger_mask = coll > 0
    if danger_mask.any():
        ax1.plot(time[danger_mask], dist[danger_mask], '.', color='red',
                 markersize=3, label='Collision')
    ax1.axhline(y=0, color='red', linestyle='--', linewidth=1, alpha=0.5, label='Collision threshold')
    ax1.set_ylabel('Min Self-Collision Dist (mm)', fontsize=11)
    ax1.set_title('HR_S50-2000 RML S-Curve Collision Simulation', fontsize=13, fontweight='bold')
    ax1.legend(loc='upper right', fontsize=9)
    ax1.grid(True, alpha=0.3)
    
    # 场景分隔线
    unique_sc = np.unique(scenario)
    colors = plt.cm.Set3(np.linspace(0, 1, len(unique_sc)))
    for sc, clr in zip(unique_sc, colors):
        mask = scenario == sc
        if mask.any():
            t_start = time[mask].min()
            ax1.axvline(x=t_start, color=clr, linestyle=':', alpha=0.5)
    
    # TCP轨迹
    ax2 = axes[1]
    ax2.plot(time, tcp_x, '-', linewidth=0.8, label='TCP_X', alpha=0.8)
    ax2.plot(time, tcp_y, '-', linewidth=0.8, label='TCP_Y', alpha=0.8)
    ax2.plot(time, tcp_z, '-', linewidth=0.8, label='TCP_Z', alpha=0.8)
    ax2.set_ylabel('TCP Position (mm)', fontsize=11)
    ax2.legend(loc='upper right', fontsize=9)
    ax2.grid(True, alpha=0.3)
    
    # 场景标记
    ax3 = axes[2]
    for sc, clr in zip(unique_sc, colors):
        mask = scenario == sc
        if mask.any():
            ax3.fill_between(time[mask], 0, 1, color=clr, alpha=0.4)
            mid_t = time[mask].mean()
            ax3.text(mid_t, 0.5, f'S{int(sc)}', ha='center', va='center', fontsize=8)
    ax3.set_ylabel('Scenario', fontsize=11)
    ax3.set_xlabel('Time (s)', fontsize=11)
    ax3.set_yticks([])
    
    plt.tight_layout()
    path = os.path.join(output_dir, 'rml_collision_analysis.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"  ✓ 碰撞分析: {path}")
    
    # 图2: TCP 3D轨迹
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    for sc, clr in zip(unique_sc, colors):
        mask = scenario == sc
        ax.plot(tcp_x[mask], tcp_y[mask], tcp_z[mask], '-',
                color=clr, linewidth=1, alpha=0.8, label=f'Scene {int(sc)}')
    ax.set_xlabel('X (mm)')
    ax.set_ylabel('Y (mm)')
    ax.set_zlabel('Z (mm)')
    ax.set_title('TCP 3D Trajectory — RML S-Curve', fontsize=13, fontweight='bold')
    ax.legend(fontsize=8)
    path = os.path.join(output_dir, 'rml_collision_tcp3d.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"  ✓ TCP轨迹: {path}")

def analyze_palletizing_data(data_dir, output_dir):
    """分析码垛仿真数据并生成图表"""
    csv_path = os.path.join(data_dir, 'rml_palletizing_profile.csv')
    scurve_path = os.path.join(data_dir, 'rml_palletizing_scurve.csv')
    summary_path = os.path.join(data_dir, 'rml_palletizing_summary.txt')
    
    # 碰撞距离
    if os.path.exists(csv_path):
        print(f"  加载码垛数据: {csv_path}")
        data = np.genfromtxt(csv_path, delimiter=',', skip_header=1, dtype=float)
        if data.ndim >= 2 and data.shape[0] > 2:
            # CSV格式: task,segment,step,time_s,q1,...,q6,selfDist_mm,selfCollision,tcpX_mm,tcpY_mm,tcpZ_mm
            task_id = data[:, 0].astype(int)
            time = data[:, 3]          # time_s
            dist = data[:, 10]         # selfDist_mm (已是mm)
            coll = data[:, 11]         # selfCollision
            tcp_x = data[:, 12]        # tcpX_mm
            tcp_y = data[:, 13]        # tcpY_mm
            tcp_z = data[:, 14]        # tcpZ_mm

            fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
            
            ax1.plot(time, dist, '-', color='#2196F3', linewidth=0.5, alpha=0.7)
            ax1.axhline(y=0, color='red', linestyle='--', linewidth=1.5, alpha=0.5)
            ax1.fill_between(time, 0, dist, where=dist > 0,
                           color='#2196F3', alpha=0.1)
            danger_mask = coll > 0
            if danger_mask.any():
                ax1.plot(time[danger_mask], dist[danger_mask], 'r.',
                        markersize=4, label='Collision')
                ax1.legend()
            ax1.set_ylabel('Min Self-Dist (mm)', fontsize=11)
            ax1.set_title('HR_S50-2000 Palletizing — Collision Distance Profile (libCmpRML.so)',
                        fontsize=13, fontweight='bold')
            ax1.grid(True, alpha=0.3)
            
            # TCP Z轨迹 (显示码垛高度变化)
            ax2.plot(time, tcp_z, '-', color='#4CAF50', linewidth=0.8, alpha=0.8)
            ax2.set_ylabel('TCP Z (mm)', fontsize=11)
            ax2.set_xlabel('Time (s)', fontsize=11)
            ax2.grid(True, alpha=0.3)
            
            # 任务换色带
            unique_tasks = np.unique(task_id)
            colors = plt.cm.tab20(np.linspace(0, 1, max(len(unique_tasks), 1)))
            for idx, (tid, clr) in enumerate(zip(unique_tasks, colors)):
                mask = task_id == tid
                if mask.any() and tid >= 0:
                    ax2.axvspan(time[mask].min(), time[mask].max(),
                              alpha=0.08, color=clr)
            
            plt.tight_layout()
            path = os.path.join(output_dir, 'rml_palletizing_analysis.png')
            plt.savefig(path, dpi=150, bbox_inches='tight')
            plt.close()
            print(f"  ✓ 码垛分析: {path}")
            
            # TCP 3D 轨迹
            fig = plt.figure(figsize=(10, 8))
            ax = fig.add_subplot(111, projection='3d')
            for tid, clr in zip(unique_tasks, colors):
                if tid < 0: continue
                mask = task_id == tid
                ax.plot(tcp_x[mask], tcp_y[mask], tcp_z[mask], '-',
                        color=clr, linewidth=0.8, alpha=0.7)
            ax.set_xlabel('X (mm)')
            ax.set_ylabel('Y (mm)')
            ax.set_zlabel('Z (mm)')
            ax.set_title('Palletizing TCP 3D Trajectory', fontsize=13, fontweight='bold')
            ax.view_init(elev=25, azim=135)
            path = os.path.join(output_dir, 'rml_palletizing_tcp3d.png')
            plt.savefig(path, dpi=150, bbox_inches='tight')
            plt.close()
            print(f"  ✓ TCP 3D: {path}")
    
    # S曲线剖面
    if os.path.exists(scurve_path):
        print(f"  加载S曲线: {scurve_path}")
        sdata = np.genfromtxt(scurve_path, delimiter=',', skip_header=1, dtype=float)
        if sdata.ndim >= 2 and sdata.shape[0] > 100:
            # 只取前2000点看清楚S曲线特征
            n = min(sdata.shape[0], 5000)
            t = sdata[:n, 0]
            
            fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
            
            joint_labels = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6']
            colors_j = ['#e53935', '#1e88e5', '#43a047', '#fb8c00', '#8e24aa', '#00acc1']
            
            for j in range(6):
                ax1.plot(t, sdata[:n, 1+j], '-', color=colors_j[j],
                         linewidth=0.8, alpha=0.8, label=joint_labels[j])
            ax1.set_ylabel('Velocity (deg/s)', fontsize=11)
            ax1.set_title('S-Curve Velocity Profile (libCmpRML.so)', fontsize=13, fontweight='bold')
            ax1.legend(fontsize=8, ncol=6, loc='upper right')
            ax1.grid(True, alpha=0.3)
            
            for j in range(6):
                ax2.plot(t, sdata[:n, 7+j], '-', color=colors_j[j],
                         linewidth=0.5, alpha=0.7, label=joint_labels[j])
            ax2.set_ylabel('Acceleration (deg/s²)', fontsize=11)
            ax2.set_xlabel('Time (s)', fontsize=11)
            ax2.legend(fontsize=8, ncol=6, loc='upper right')
            ax2.grid(True, alpha=0.3)
            
            plt.tight_layout()
            path = os.path.join(output_dir, 'rml_scurve_profile.png')
            plt.savefig(path, dpi=150, bbox_inches='tight')
            plt.close()
            print(f"  ✓ S曲线: {path}")
    
    # 节拍统计
    if os.path.exists(summary_path):
        print(f"  加载摘要: {summary_path}")
        cycles = []
        with open(summary_path, 'r') as f:
            for line in f:
                if line.startswith('cycle '):
                    parts = line.strip().split()
                    if len(parts) >= 4:
                        cycles.append({
                            'id': int(parts[1]),
                            'label': parts[2],
                            'time': float(parts[3]),
                            'dist': float(parts[4]) if len(parts) > 4 else 0,
                            'safe': int(parts[5]) if len(parts) > 5 else 1
                        })
        
        if cycles:
            fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
            
            labels = [c['label'] for c in cycles]
            times = [c['time'] for c in cycles]
            dists = [c['dist']*1000 for c in cycles]
            safe_colors = ['#4CAF50' if c['safe'] else '#F44336' for c in cycles]
            
            bars = ax1.bar(range(len(times)), times, color=safe_colors, alpha=0.8,
                          edgecolor='white', linewidth=0.5)
            ax1.set_xlabel('Task', fontsize=11)
            ax1.set_ylabel('Cycle Time (s)', fontsize=11)
            ax1.set_title('Palletizing Cycle Times', fontsize=13, fontweight='bold')
            ax1.set_xticks(range(len(labels)))
            ax1.set_xticklabels(labels, rotation=45, ha='right', fontsize=8)
            avg_t = np.mean(times)
            ax1.axhline(y=avg_t, color='#FF9800', linestyle='--', linewidth=1.5,
                       label=f'Avg: {avg_t:.3f}s')
            ax1.legend(fontsize=10)
            ax1.grid(True, alpha=0.3, axis='y')
            
            ax2.bar(range(len(dists)), dists, color='#2196F3', alpha=0.8,
                   edgecolor='white', linewidth=0.5)
            ax2.axhline(y=0, color='red', linestyle='--', linewidth=1.5)
            ax2.set_xlabel('Task', fontsize=11)
            ax2.set_ylabel('Min Self-Distance (mm)', fontsize=11)
            ax2.set_title('Collision Safety Margin', fontsize=13, fontweight='bold')
            ax2.set_xticks(range(len(labels)))
            ax2.set_xticklabels(labels, rotation=45, ha='right', fontsize=8)
            ax2.grid(True, alpha=0.3, axis='y')
            
            plt.tight_layout()
            path = os.path.join(output_dir, 'rml_palletizing_cycles.png')
            plt.savefig(path, dpi=150, bbox_inches='tight')
            plt.close()
            print(f"  ✓ 节拍统计: {path}")

# ============================================================================
# 机器人多姿态对比图 (STL网格)
# ============================================================================
def plot_robot_poses(stl_dir, output_dir):
    """生成多姿态对比3D图"""
    poses_deg = {
        'HOME': [0, -90, 0, 0, 90, 0],
        'Pick': [35, -45, 45, 0, 45, 35],
        'Place-L0': [-40, -55, 35, 0, 55, -40],
        'Place-L2': [-40, -47, 35, 0, 55, -40],
        'Extended': [0, -170, 10, 0, 0, 0],
        'Folded': [0, -10, -140, 180, 0, 0],
    }
    
    fig = plt.figure(figsize=(18, 10))
    
    for idx, (name, q_deg) in enumerate(poses_deg.items()):
        ax = fig.add_subplot(2, 3, idx+1, projection='3d')
        
        try:
            positions = plot_robot_stl(ax, q_deg, stl_dir, alpha=0.5)
        except Exception as e:
            print(f"  [警告] 无法绘制 {name}: {e}")
            # Fallback: 骨架图
            q_rad = np.deg2rad(q_deg)
            transforms = fk_dh(q_rad)
            pts = [T[:3, 3] * 1000 for T in transforms]
            xs = [p[0] for p in pts]
            ys = [p[1] for p in pts]
            zs = [p[2] for p in pts]
            ax.plot(xs, ys, zs, 'o-', color='#1976D2', linewidth=3, markersize=8)
        
        ax.set_title(f'{name}\n{q_deg}°', fontsize=10, fontweight='bold')
        ax.set_xlabel('X (mm)', fontsize=8)
        ax.set_ylabel('Y (mm)', fontsize=8)
        ax.set_zlabel('Z (mm)', fontsize=8)
        ax.set_xlim([-1500, 1500])
        ax.set_ylim([-1500, 1500])
        ax.set_zlim([-500, 2200])
        ax.view_init(elev=25, azim=135)
    
    fig.suptitle('HR_S50-2000 Multi-Pose Visualization (STL Mesh)',
                 fontsize=14, fontweight='bold')
    plt.tight_layout()
    path = os.path.join(output_dir, 'rml_s50_poses_stl.png')
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"  ✓ 姿态对比: {path}")

# ============================================================================
# 主函数
# ============================================================================
def main():
    print("╔══════════════════════════════════════════════════════════╗")
    print("║   HR_S50-2000 STL 3D 仿真可视化分析                     ║")
    print("║   URDF/STL mesh + libCmpRML.so S曲线 结果分析            ║")
    print("╚══════════════════════════════════════════════════════════╝\n")
    
    # 路径设置
    project_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    stl_dir = os.path.join(project_dir, 'ArmCollisionModel', 'model', 'meshes', 'S50')
    data_dir = os.path.join(project_dir, 'data')
    output_dir = os.path.join(data_dir, 'sim3d')
    
    os.makedirs(output_dir, exist_ok=True)
    
    print(f"  项目目录: {project_dir}")
    print(f"  STL目录:  {stl_dir}")
    print(f"  数据目录: {data_dir}")
    print(f"  输出目录: {output_dir}\n")
    
    # 1. 机器人多姿态 STL 可视化
    print("━━━ 1. 机器人多姿态 STL 可视化 ━━━━━━━━━━━━━━━━━━━━━━━━━━")
    if os.path.exists(stl_dir):
        stl_files = [f for f in os.listdir(stl_dir) if f.endswith('.STL')]
        print(f"  找到 {len(stl_files)} 个 STL 网格文件")
        plot_robot_poses(stl_dir, output_dir)
    else:
        print(f"  [跳过] STL目录不存在: {stl_dir}")
    
    # 2. 碰撞仿真数据分析
    print("\n━━━ 2. 碰撞仿真数据分析 ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
    analyze_collision_data(data_dir, output_dir)
    
    # 3. 码垛仿真数据分析
    print("\n━━━ 3. 码垛仿真数据分析 ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
    analyze_palletizing_data(data_dir, output_dir)
    
    print("\n══════════════════════════════════════════════════════════════")
    print("  可视化完成! 输出目录: data/sim3d/")
    print("══════════════════════════════════════════════════════════════")

if __name__ == '__main__':
    main()
