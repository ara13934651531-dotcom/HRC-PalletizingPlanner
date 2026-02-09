"""分析 PalletizingSimApp v2.0 截屏, 生成带注释的分析图"""
import os
from PIL import Image, ImageDraw, ImageFont
import json

img_dir = '/home/ara/桌面/X86_test/ArmCollisionModel/pic/sim_test'
out_dir = os.path.join(img_dir, 'analysis')
os.makedirs(out_dir, exist_ok=True)

files = sorted([f for f in os.listdir(img_dir) if f.endswith('.png') and not f.startswith('analysis')])
print(f"Found {len(files)} screenshots")

# 逐张分析
for f in files:
    path = os.path.join(img_dir, f)
    img = Image.open(path)
    w, h = img.size
    print(f"\n=== {f} ({w}x{h}) ===")
    
    # 分析3D场景区域 (左侧约62%)
    scene_region = img.crop((0, 0, int(w*0.62), h))
    # 分析右侧面板区域
    panel_region = img.crop((int(w*0.63), 0, w, h))
    
    # 颜色分析 - 检查3D场景主要颜色
    scene_colors = scene_region.resize((50, 50)).getcolors(maxcolors=500)
    if scene_colors:
        scene_colors.sort(key=lambda x: -x[0])
        print(f"  Scene dominant colors (top 5):")
        for cnt, col in scene_colors[:5]:
            print(f"    count={cnt}, RGB={col}")
    
    # 检查面板区域文字
    panel_colors = panel_region.resize((30, 30)).getcolors(maxcolors=200)
    if panel_colors:
        panel_colors.sort(key=lambda x: -x[0])
        print(f"  Panel dominant colors (top 3):")
        for cnt, col in panel_colors[:3]:
            print(f"    count={cnt}, RGB={col}")
    
    # 保存缩略图便于查看
    thumb = img.resize((w//2, h//2), Image.LANCZOS)
    thumb.save(os.path.join(out_dir, f'thumb_{f}'))

# 详细区域分析
print("\n\n=== DETAILED REGION ANALYSIS ===")
for f in ['01_initial.png', '04_box3_added.png', '05_after_execute.png', '07_top_view.png']:
    path = os.path.join(img_dir, f)
    if not os.path.exists(path):
        continue
    img = Image.open(path)
    w, h = img.size
    print(f"\n--- {f} ---")
    
    # 检查右侧面板不同区域
    # Scene Editor panel (~68%-99% height)
    scn_panel = img.crop((int(w*0.635), int(h*0.01), w, int(h*0.32)))
    # Robot Control panel (~34%-67%)
    rob_panel = img.crop((int(w*0.635), int(h*0.33), w, int(h*0.67)))
    # Task Control panel (~4%-33%)  
    task_panel = img.crop((int(w*0.635), int(h*0.67), w, int(h*0.96)))
    # Status bar
    status_bar = img.crop((int(w*0.01), int(h*0.97), int(w*0.99), h))
    
    for name, region in [('SceneEditor', scn_panel), ('RobotCtrl', rob_panel), 
                          ('TaskCtrl', task_panel), ('StatusBar', status_bar)]:
        rw, rh = region.size
        colors = region.resize((20, 20)).getcolors(maxcolors=200)
        if colors:
            colors.sort(key=lambda x: -x[0])
            dark_pixels = sum(c for c, rgb in colors if all(v < 80 for v in rgb[:3]))
            light_pixels = sum(c for c, rgb in colors)
            print(f"  {name}: {rw}x{rh}, dark_ratio={dark_pixels/max(1,light_pixels):.2f}")
    
    # 检查3D场景中机器人是否可见 (查找特定颜色区域)
    scene = img.crop((int(w*0.05), int(h*0.05), int(w*0.60), int(h*0.95)))
    sw, sh = scene.size
    
    # 查找红色TCP标记
    red_count = 0
    green_count = 0
    blue_robot_count = 0
    orange_trail = 0
    brown_box = 0
    
    px = scene.load()
    sample_step = 4  # 采样步长
    for y in range(0, sh, sample_step):
        for x in range(0, sw, sample_step):
            r, g, b = px[x, y][:3]
            if r > 200 and g < 80 and b < 80:
                red_count += 1
            if g > 200 and r < 80 and b < 80:
                green_count += 1
            if b > 150 and r < 100 and g < 120 and b > r + 50:
                blue_robot_count += 1
            if r > 200 and 50 < g < 120 and b < 50:
                orange_trail += 1
            if 120 < r < 200 and 80 < g < 160 and 40 < b < 100:
                brown_box += 1
    
    print(f"  3D Scene pixel analysis (step={sample_step}):")
    print(f"    Red (TCP marker): {red_count}")
    print(f"    Green (Pick marker): {green_count}")
    print(f"    Blue-ish (robot/pallet): {blue_robot_count}")
    print(f"    Orange (trail line): {orange_trail}")
    print(f"    Brown (boxes): {brown_box}")

print("\n=== Analysis complete ===")
