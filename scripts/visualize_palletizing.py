#!/usr/bin/env python3
"""
码垛机器人路径可视化脚本
HR_S50-2000 协作机器人运动规划可视化

功能:
1. 3D可视化机器人工作空间
2. 动画展示规划路径
3. 对比原始路径和优化路径
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
import os

# DH参数 (mm -> m)
DH_PARAMS = {
    'd1': 0.2965,
    'd2': 0.3362,
    'd3': 0.2390,
    'd4': 0.1585,
    'd5': 0.1585,
    'd6': 0.1345,
    'a2': 0.900,
    'a3': 0.9415
}

def dh_transform(theta, d, a, alpha):
    """计算DH变换矩阵"""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    
    return np.array([
        [ct, -st*ca, st*sa, a*ct],
        [st, ct*ca, -ct*sa, a*st],
        [0, sa, ca, d],
        [0, 0, 0, 1]
    ])

def forward_kinematics(q):
    """
    正运动学 - 计算各关节位置
    q: 6个关节角度 (弧度)
    返回: 7个位置点 (基座 + 6个关节)
    """
    d = DH_PARAMS
    
    # 简化的DH模型 (UR类型)
    positions = [np.array([0, 0, 0])]  # 基座
    
    T = np.eye(4)
    
    # Link 1
    T1 = dh_transform(q[0], d['d1'], 0, np.pi/2)
    T = T @ T1
    positions.append(T[:3, 3].copy())
    
    # Link 2
    T2 = dh_transform(q[1], d['d2'], d['a2'], 0)
    T = T @ T2
    positions.append(T[:3, 3].copy())
    
    # Link 3
    T3 = dh_transform(q[2], d['d3'], d['a3'], 0)
    T = T @ T3
    positions.append(T[:3, 3].copy())
    
    # Link 4
    T4 = dh_transform(q[3], d['d4'], 0, np.pi/2)
    T = T @ T4
    positions.append(T[:3, 3].copy())
    
    # Link 5
    T5 = dh_transform(q[4], d['d5'], 0, -np.pi/2)
    T = T @ T5
    positions.append(T[:3, 3].copy())
    
    # Link 6 (TCP)
    T6 = dh_transform(q[5], d['d6'], 0, 0)
    T = T @ T6
    positions.append(T[:3, 3].copy())
    
    return np.array(positions)

def load_path(filename):
    """加载路径文件"""
    if not os.path.exists(filename):
        print(f"警告: 文件 {filename} 不存在")
        return None
    
    data = []
    with open(filename, 'r') as f:
        for line in f:
            line = line.strip()
            if line.startswith('#') or not line:
                continue
            parts = line.split()
            if len(parts) >= 7:
                # 跳过索引列，读取6个关节角度
                q = [float(parts[i]) for i in range(1, 7)]
                data.append(q)
    
    return np.array(data) if data else None

def compute_tcp_path(joint_path):
    """计算TCP轨迹"""
    tcp_positions = []
    for q_deg in joint_path:
        q_rad = np.radians(q_deg)
        positions = forward_kinematics(q_rad)
        tcp_positions.append(positions[-1])  # TCP位置
    return np.array(tcp_positions)

def plot_robot(ax, q_rad, color='b', alpha=1.0, linewidth=3):
    """绘制机器人连杆"""
    positions = forward_kinematics(q_rad)
    
    # 绘制连杆
    xs = positions[:, 0]
    ys = positions[:, 1]
    zs = positions[:, 2]
    
    ax.plot(xs, ys, zs, 'o-', color=color, linewidth=linewidth, 
            markersize=8, alpha=alpha)
    
    # 绘制关节
    ax.scatter(xs, ys, zs, c=color, s=50, alpha=alpha)
    
    return positions

def plot_workspace(ax):
    """绘制工作空间和障碍物"""
    # 地面网格
    xx, yy = np.meshgrid(np.linspace(-2, 2, 10), np.linspace(-2, 2, 10))
    zz = np.zeros_like(xx)
    ax.plot_surface(xx, yy, zz, alpha=0.1, color='gray')
    
    # 集装箱区域 (简化为立方体框架)
    def draw_box(center, size, color='b', alpha=0.3):
        x, y, z = center
        l, w, h = size
        
        # 8个顶点
        vertices = np.array([
            [x, y, z], [x+l, y, z], [x+l, y+w, z], [x, y+w, z],
            [x, y, z+h], [x+l, y, z+h], [x+l, y+w, z+h], [x, y+w, z+h]
        ])
        
        # 12条边
        edges = [
            [0, 1], [1, 2], [2, 3], [3, 0],  # 底面
            [4, 5], [5, 6], [6, 7], [7, 4],  # 顶面
            [0, 4], [1, 5], [2, 6], [3, 7]   # 侧边
        ]
        
        for edge in edges:
            ax.plot3D(*vertices[edge].T, color=color, alpha=alpha, linewidth=1)
    
    # 绘制码垛区域 (蓝色框架)
    draw_box([-0.8-0.6, 0.8-0.5, 0], [1.2, 1.0, 2.0], 'blue', 0.5)
    
    # 绘制流水线
    draw_box([0.8-1.0, -0.3, 0], [2.0, 0.6, 0.8], 'green', 0.3)
    
    # 机器人基座
    ax.scatter([0], [0], [0], c='red', s=200, marker='^', label='Robot Base')

def main():
    """主函数"""
    print("=" * 60)
    print("HR_S50-2000 码垛机器人路径可视化")
    print("=" * 60)
    
    # 加载路径数据
    path_files = {
        'raw': 'path_raw.txt',
        'optimized': 'path_optimized.txt',
        'spline': 'path_spline.txt',
        'palletizing': 'palletizing_path.txt'
    }
    
    paths = {}
    for name, filename in path_files.items():
        path = load_path(filename)
        if path is not None:
            paths[name] = path
            print(f"加载 {name}: {len(path)} 个路径点")
    
    if not paths:
        print("没有找到路径文件，请先运行 testPalletizingPlanner")
        return
    
    # 创建图形
    fig = plt.figure(figsize=(16, 6))
    
    # 子图1: 路径对比
    ax1 = fig.add_subplot(131, projection='3d')
    ax1.set_title('Path Comparison')
    
    plot_workspace(ax1)
    
    colors = {'raw': 'red', 'optimized': 'green', 'spline': 'blue'}
    
    for name, path in paths.items():
        if name in colors:
            tcp_path = compute_tcp_path(path)
            ax1.plot(tcp_path[:, 0], tcp_path[:, 1], tcp_path[:, 2],
                    '-', color=colors[name], linewidth=2, label=name, alpha=0.8)
    
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_zlabel('Z (m)')
    ax1.legend()
    
    # 子图2: 机器人姿态序列
    ax2 = fig.add_subplot(132, projection='3d')
    ax2.set_title('Robot Poses Along Path')
    
    plot_workspace(ax2)
    
    # 选择一条路径显示
    display_path = paths.get('optimized', paths.get('raw'))
    if display_path is not None:
        n = len(display_path)
        indices = np.linspace(0, n-1, min(8, n), dtype=int)
        
        for i, idx in enumerate(indices):
            q_rad = np.radians(display_path[idx])
            alpha = 0.3 + 0.7 * i / len(indices)
            plot_robot(ax2, q_rad, color=plt.cm.viridis(i / len(indices)), 
                      alpha=alpha, linewidth=2)
        
        tcp_path = compute_tcp_path(display_path)
        ax2.plot(tcp_path[:, 0], tcp_path[:, 1], tcp_path[:, 2],
                '--', color='orange', linewidth=1.5, alpha=0.5)
    
    ax2.set_xlabel('X (m)')
    ax2.set_ylabel('Y (m)')
    ax2.set_zlabel('Z (m)')
    
    # 子图3: 关节角度变化
    ax3 = fig.add_subplot(133)
    ax3.set_title('Joint Angles vs Path Parameter')
    
    if display_path is not None:
        t = np.linspace(0, 1, len(display_path))
        joint_names = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6']
        colors = plt.cm.tab10(np.linspace(0, 1, 6))
        
        for i, (name, color) in enumerate(zip(joint_names, colors)):
            ax3.plot(t, display_path[:, i], '-', color=color, 
                    linewidth=1.5, label=name)
    
    ax3.set_xlabel('Path Parameter')
    ax3.set_ylabel('Joint Angle (deg)')
    ax3.legend(loc='upper right', ncol=2)
    ax3.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    # 保存图像
    output_file = 'palletizing_path_visualization.png'
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"\n可视化已保存到: {output_file}")
    
    plt.show()
    
    # 打印路径统计
    print("\n" + "=" * 60)
    print("路径统计:")
    print("=" * 60)
    
    for name, path in paths.items():
        tcp_path = compute_tcp_path(path)
        
        # 计算TCP路径长度
        tcp_length = np.sum(np.linalg.norm(np.diff(tcp_path, axis=0), axis=1))
        
        # 计算关节空间路径长度
        joint_length = np.sum(np.linalg.norm(np.diff(np.radians(path), axis=0), axis=1))
        
        print(f"\n{name}:")
        print(f"  路径点数: {len(path)}")
        print(f"  TCP路径长度: {tcp_length*1000:.1f} mm")
        print(f"  关节空间长度: {joint_length:.4f} rad")

if __name__ == '__main__':
    main()
