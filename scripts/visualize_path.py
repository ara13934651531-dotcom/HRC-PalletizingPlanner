#!/usr/bin/env python3
"""
码垛路径可视化脚本
用于展示规划结果和B-Spline平滑效果
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os

# 设置中文字体
plt.rcParams['font.sans-serif'] = ['WenQuanYi Micro Hei', 'SimHei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

def load_path_file(filename):
    """加载路径文件"""
    data = []
    with open(filename, 'r') as f:
        for line in f:
            line = line.strip()
            if line.startswith('#') or not line:
                continue
            values = list(map(float, line.split()))
            if len(values) >= 6:
                # 如果有7列，第一列是t或index
                data.append(values[-6:] if len(values) > 6 else values)
    return np.array(data)

def forward_kinematics_simple(q, dh_params):
    """简化的正运动学计算（仅用于可视化）"""
    # HR_S50-2000 DH参数 (mm)
    d1, d2, d3, d4, d5, d6, a2, a3 = dh_params
    
    # 转换为米
    d1, d2, d3, d4, d5, d6 = [x/1000 for x in [d1, d2, d3, d4, d5, d6]]
    a2, a3 = a2/1000, a3/1000
    
    # 简化计算 - 仅计算末端位置
    c1, s1 = np.cos(q[0]), np.sin(q[0])
    c2, s2 = np.cos(q[1]), np.sin(q[1])
    c23 = np.cos(q[1] + q[2])
    s23 = np.sin(q[1] + q[2])
    
    # 近似末端位置
    x = c1 * (a2 * c2 + a3 * c23) + d4 * c1 * c23
    y = s1 * (a2 * c2 + a3 * c23) + d4 * s1 * c23
    z = d1 + a2 * s2 + a3 * s23 + d4 * s23
    
    return np.array([x, y, z])

def plot_joint_space(ax, path_data, title, color='b'):
    """绘制关节空间轨迹"""
    t = np.linspace(0, 1, len(path_data))
    
    for i in range(6):
        ax.plot(t, np.degrees(path_data[:, i]), label=f'J{i+1}', alpha=0.8)
    
    ax.set_xlabel('归一化时间')
    ax.set_ylabel('关节角度 (度)')
    ax.set_title(title)
    ax.legend(loc='best', ncol=3)
    ax.grid(True, alpha=0.3)

def plot_cartesian_path(ax, path_data, dh_params, title, color='b'):
    """绘制笛卡尔空间路径"""
    positions = []
    for q in path_data:
        pos = forward_kinematics_simple(q, dh_params)
        positions.append(pos)
    positions = np.array(positions)
    
    ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], 
            color=color, linewidth=2, label='路径')
    ax.scatter(positions[0, 0], positions[0, 1], positions[0, 2], 
               c='g', s=100, marker='o', label='起点')
    ax.scatter(positions[-1, 0], positions[-1, 1], positions[-1, 2], 
               c='r', s=100, marker='s', label='终点')
    
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title(title)
    ax.legend()

def plot_curvature(ax, path_data, title):
    """计算并绘制路径曲率"""
    # 数值计算一阶和二阶导数
    dt = 1.0 / (len(path_data) - 1)
    
    # 一阶导数
    dq = np.gradient(path_data, dt, axis=0)
    
    # 二阶导数
    ddq = np.gradient(dq, dt, axis=0)
    
    # 计算曲率 (高维推广)
    curvatures = []
    for i in range(len(path_data)):
        d1 = dq[i]
        d2 = ddq[i]
        d1_norm = np.linalg.norm(d1)
        
        if d1_norm < 1e-9:
            curvatures.append(0)
            continue
        
        # 投影
        d2_perp = d2 - (np.dot(d1, d2) / (d1_norm**2)) * d1
        curvature = np.linalg.norm(d2_perp) / (d1_norm**2)
        curvatures.append(curvature)
    
    t = np.linspace(0, 1, len(curvatures))
    ax.plot(t, curvatures, 'b-', linewidth=1.5)
    ax.set_xlabel('归一化时间')
    ax.set_ylabel('曲率')
    ax.set_title(title)
    ax.grid(True, alpha=0.3)

def main():
    # 切换到项目目录
    project_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    os.chdir(project_dir)
    
    # HR_S50-2000 DH参数
    dh_params = [296.5, 336.2, 239.0, 158.5, 158.5, 134.5, 900.0, 941.5]
    
    # 尝试加载不同的路径文件
    files_to_load = [
        ('path_raw.txt', '原始RRT*路径'),
        ('path_optimized.txt', '优化后路径'),
        ('path_spline.txt', 'B-Spline平滑路径'),
        ('palletizing_spline.txt', '码垛任务路径')
    ]
    
    loaded_paths = []
    for filename, desc in files_to_load:
        if os.path.exists(filename):
            data = load_path_file(filename)
            if len(data) > 0:
                loaded_paths.append((filename, desc, data))
                print(f"加载: {filename} ({len(data)} 个点)")
    
    if not loaded_paths:
        print("未找到任何路径文件，请先运行 testPalletizingPlanner")
        return
    
    # 创建图形
    fig = plt.figure(figsize=(16, 12))
    fig.suptitle('HR_S50-2000 协作机器人码垛路径规划结果', fontsize=14, fontweight='bold')
    
    # 绘制各个路径的关节空间轨迹
    num_paths = len(loaded_paths)
    colors = ['b', 'g', 'r', 'orange', 'purple']
    
    # 子图1：关节空间比较
    ax1 = fig.add_subplot(2, 2, 1)
    for i, (_, desc, data) in enumerate(loaded_paths):
        t = np.linspace(0, 1, len(data))
        # 只显示第一个关节作为代表
        ax1.plot(t, np.degrees(data[:, 0]), color=colors[i % len(colors)], 
                 label=f'{desc} (J1)', alpha=0.8)
    ax1.set_xlabel('归一化时间')
    ax1.set_ylabel('关节角度 (度)')
    ax1.set_title('关节1轨迹比较')
    ax1.legend(loc='best')
    ax1.grid(True, alpha=0.3)
    
    # 子图2：3D笛卡尔路径
    ax2 = fig.add_subplot(2, 2, 2, projection='3d')
    for i, (_, desc, data) in enumerate(loaded_paths[-2:] if len(loaded_paths) >= 2 else loaded_paths):
        positions = []
        for q in data[::max(1, len(data)//50)]:  # 降采样以提高性能
            pos = forward_kinematics_simple(q, dh_params)
            positions.append(pos)
        positions = np.array(positions)
        
        ax2.plot(positions[:, 0], positions[:, 1], positions[:, 2], 
                 color=colors[i % len(colors)], linewidth=1.5, label=desc, alpha=0.8)
    
    ax2.set_xlabel('X (m)')
    ax2.set_ylabel('Y (m)')
    ax2.set_zlabel('Z (m)')
    ax2.set_title('笛卡尔空间路径')
    ax2.legend()
    
    # 子图3：最终路径的所有关节
    ax3 = fig.add_subplot(2, 2, 3)
    final_path = loaded_paths[-1][2]
    plot_joint_space(ax3, final_path, f'最终路径关节轨迹 ({loaded_paths[-1][1]})')
    
    # 子图4：曲率分析
    ax4 = fig.add_subplot(2, 2, 4)
    plot_curvature(ax4, final_path, '路径曲率分析')
    
    plt.tight_layout()
    plt.savefig('path_visualization.png', dpi=150, bbox_inches='tight')
    print("\n可视化图已保存到: path_visualization.png")
    plt.show()

if __name__ == '__main__':
    main()
