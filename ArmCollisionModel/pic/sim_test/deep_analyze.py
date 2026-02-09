"""深度分析 PalletizingSimApp v2.0 截屏 — 逐区域检查问题"""
import os
from PIL import Image, ImageDraw, ImageFont
import numpy as np

img_dir = '/home/ara/桌面/X86_test/ArmCollisionModel/pic/sim_test'
out_dir = os.path.join(img_dir, 'analysis')
os.makedirs(out_dir, exist_ok=True)

issues = []

def add_issue(severity, category, desc):
    issues.append({'severity': severity, 'category': category, 'desc': desc})
    print(f"  [{severity}] {category}: {desc}")

# =============================================================================
# 分析 01_initial.png - 初始界面
# =============================================================================
print("=" * 60)
print("ANALYZING 01_initial.png - Initial Interface")
print("=" * 60)
img1 = Image.open(os.path.join(img_dir, '01_initial.png'))
arr1 = np.array(img1)
h, w = arr1.shape[:2]
print(f"Image size: {w}x{h}")

# 1) 检查3D场景区域是否有合理内容
scene_arr = arr1[:, :int(w*0.62), :]
# 检查场景是否太暗或太亮
mean_brightness = scene_arr.mean()
print(f"  Scene mean brightness: {mean_brightness:.1f}")
if mean_brightness < 80:
    add_issue('HIGH', 'Rendering', 'Scene is too dark, possible lighting issue')
elif mean_brightness > 240:
    add_issue('MEDIUM', 'Rendering', 'Scene is too bright/washed out')

# 2) 检查右侧面板区域
panel_arr = arr1[:, int(w*0.635):, :]
panel_mean = panel_arr.mean()
print(f"  Panel mean brightness: {panel_mean:.1f}")

# 3) 检查状态栏
status_arr = arr1[int(h*0.97):, :, :]
status_mean = status_arr.mean(axis=(0,1))
print(f"  Status bar mean RGB: ({status_mean[0]:.0f}, {status_mean[1]:.0f}, {status_mean[2]:.0f})")

# 4) 检查Pick/Place标记 - 初始时不应有绿色Pick标记...除非autoFillPlace被调用
# 检查红色三角(Place标记)
red_mask = (arr1[:,:,0] > 180) & (arr1[:,:,1] < 60) & (arr1[:,:,2] < 60)
red_count = red_mask.sum()
print(f"  Red pixels (Place mark + TCP): {red_count}")

green_mask = (arr1[:,:,1] > 180) & (arr1[:,:,0] < 60) & (arr1[:,:,2] < 60)
green_count = green_mask.sum()
print(f"  Green pixels (Pick mark): {green_count}")

# 5) 检查机器人胶囊体是否可见
# 机器人胶囊体颜色: 蓝灰色 [0.75 0.75 0.80], [0.50 0.55 0.85] 等
robot_blue_mask = (arr1[:,:,2] > 130) & (arr1[:,:,2] > arr1[:,:,0] + 30) & (arr1[:,:,2] > arr1[:,:,1] + 20)
robot_vis = robot_blue_mask[:int(h*0.95), :int(w*0.62)].sum()
print(f"  Robot-blue pixels in scene: {robot_vis}")
if robot_vis < 100:
    add_issue('HIGH', 'Rendering', 'Robot capsules not visible in scene')

# =============================================================================
# 分析 04_box3_added.png - 三个箱子
# =============================================================================
print("\n" + "=" * 60)
print("ANALYZING 04_box3_added.png - Three Boxes Added")
print("=" * 60)
img4 = Image.open(os.path.join(img_dir, '04_box3_added.png'))
arr4 = np.array(img4)

# 检查棕色箱子区域
brown_mask_4 = (arr4[:,:,0] > 120) & (arr4[:,:,0] < 210) & \
               (arr4[:,:,1] > 80) & (arr4[:,:,1] < 170) & \
               (arr4[:,:,2] > 30) & (arr4[:,:,2] < 110) & \
               (arr4[:,:,0] > arr4[:,:,2] + 30)
brown_scene = brown_mask_4[:int(h*0.95), :int(w*0.62)]
brown_count_4 = brown_scene.sum()
print(f"  Brown/box pixels in scene: {brown_count_4}")

# 找到箱子的位置分布 (Y方向应该不同)
if brown_count_4 > 500:
    ys, xs = np.where(brown_scene)
    y_unique = np.unique(ys)
    x_unique = np.unique(xs)
    print(f"  Box pixel Y range: {ys.min()}-{ys.max()} (span={ys.max()-ys.min()})")
    print(f"  Box pixel X range: {xs.min()}-{xs.max()} (span={xs.max()-xs.min()})")
    
    # 检查箱子是否在不同位置 (用Y聚类)
    y_hist, y_bins = np.histogram(ys, bins=20)
    peaks = np.where(y_hist > brown_count_4 * 0.02)[0]
    print(f"  Box Y distribution peaks: {len(peaks)} clusters")
    if len(peaks) <= 2:
        add_issue('MEDIUM', 'Layout', 'Boxes might be overlapping or too close together')

# 检查Pick(绿色)和Place(红色)标记
green_4 = ((arr4[:,:,1] > 180) & (arr4[:,:,0] < 100) & (arr4[:,:,2] < 100))
green_count_4 = green_4[:int(h*0.95), :int(w*0.62)].sum()
print(f"  Green (Pick) marker pixels: {green_count_4}")

red_marker = ((arr4[:,:,0] > 180) & (arr4[:,:,1] < 80) & (arr4[:,:,2] < 80))
red_count_4 = red_marker[:int(h*0.95), :int(w*0.62)].sum()
print(f"  Red (Place/TCP) marker pixels: {red_count_4}")

# =============================================================================
# 分析 05_after_execute.png - 执行后
# =============================================================================
print("\n" + "=" * 60)
print("ANALYZING 05_after_execute.png - After Execution")
print("=" * 60)
img5 = Image.open(os.path.join(img_dir, '05_after_execute.png'))
arr5 = np.array(img5)

# 轨迹线 (橙色)
orange_mask = (arr5[:,:,0] > 200) & (arr5[:,:,1] > 50) & (arr5[:,:,1] < 130) & (arr5[:,:,2] < 50)
orange_scene = orange_mask[:int(h*0.95), :int(w*0.62)]
orange_count = orange_scene.sum()
print(f"  Orange trail pixels: {orange_count}")
if orange_count > 100:
    oys, oxs = np.where(orange_scene)
    print(f"  Trail Y range: {oys.min()}-{oys.max()}")
    print(f"  Trail X range: {oxs.min()}-{oxs.max()}")
else:
    add_issue('HIGH', 'Animation', 'No visible trail line after execution')

# 检查码垛区域是否有放置的箱子
# 码垛区域大约在3D场景的中上部分 (pallet在Y方向偏上)
pallet_region = arr5[int(h*0.2):int(h*0.7), int(w*0.1):int(w*0.45), :]
dark_brown = (pallet_region[:,:,0] > 100) & (pallet_region[:,:,0] < 180) & \
             (pallet_region[:,:,1] > 60) & (pallet_region[:,:,1] < 130) & \
             (pallet_region[:,:,2] > 20) & (pallet_region[:,:,2] < 90)
placed_box_px = dark_brown.sum()
print(f"  Placed box pixels in pallet region: {placed_box_px}")

# =============================================================================
# 分析 07_top_view.png - 俯视图 (最能看出布局问题)
# =============================================================================
print("\n" + "=" * 60)
print("ANALYZING 07_top_view.png - Top View (Layout Check)")
print("=" * 60)
img7 = Image.open(os.path.join(img_dir, '07_top_view.png'))
arr7 = np.array(img7)
scene7 = arr7[:int(h*0.95), :int(w*0.62), :]

# 俯视图中检查各元素分布
# 蓝色 (pallet/frame)
blue7 = (scene7[:,:,2] > 130) & (scene7[:,:,2] > scene7[:,:,0] + 20) & (scene7[:,:,2] > scene7[:,:,1] + 10)
blue_count_7 = blue7.sum()
print(f"  Blue (pallet/frame) pixels: {blue_count_7}")

# 灰色 (conveyor)
dark_gray = (scene7[:,:,0] > 50) & (scene7[:,:,0] < 100) & \
            (np.abs(scene7[:,:,0].astype(int) - scene7[:,:,1].astype(int)) < 15) & \
            (np.abs(scene7[:,:,1].astype(int) - scene7[:,:,2].astype(int)) < 15)
gray_count = dark_gray.sum()
print(f"  Dark gray (conveyor) pixels: {gray_count}")

# 棕色 (boxes on conveyor)
brown7 = (scene7[:,:,0] > 120) & (scene7[:,:,0] < 210) & \
         (scene7[:,:,1] > 80) & (scene7[:,:,1] < 170) & \
         (scene7[:,:,2] > 30) & (scene7[:,:,2] < 110) & \
         (scene7[:,:,0] > scene7[:,:,2] + 30)
brown_count_7 = brown7.sum()
print(f"  Brown (boxes) pixels: {brown_count_7}")

# 橙色轨迹
orange7 = (scene7[:,:,0] > 200) & (scene7[:,:,1] > 50) & (scene7[:,:,1] < 130) & (scene7[:,:,2] < 50)
orange_count_7 = orange7.sum()
print(f"  Orange (trail) pixels: {orange_count_7}")

if orange_count_7 > 100:
    oys7, oxs7 = np.where(orange7)
    print(f"  Trail covers X range: {oxs7.min()}-{oxs7.max()} ({oxs7.max()-oxs7.min()} px)")
    print(f"  Trail covers Y range: {oys7.min()}-{oys7.max()} ({oys7.max()-oys7.min()} px)")

# =============================================================================
# 分析 06_front_view.png - 正面视角
# =============================================================================
print("\n" + "=" * 60)
print("ANALYZING 06_front_view.png - Front View")
print("=" * 60)
img6 = Image.open(os.path.join(img_dir, '06_front_view.png'))
arr6 = np.array(img6)
scene6 = arr6[:int(h*0.95), :int(w*0.62), :]

# 正面视角能看到机器人、框架高度
blue6 = (scene6[:,:,2] > 130) & (scene6[:,:,2] > scene6[:,:,0] + 20)
if blue6.sum() > 100:
    bys, bxs = np.where(blue6)
    print(f"  Blue elements Y range: {bys.min()}-{bys.max()} (height span)")
    print(f"  Blue elements X range: {bxs.min()}-{bxs.max()}")

orange6 = (scene6[:,:,0] > 200) & (scene6[:,:,1] > 50) & (scene6[:,:,1] < 130) & (scene6[:,:,2] < 50)
if orange6.sum() > 100:
    oys6, oxs6 = np.where(orange6)
    print(f"  Trail Y range: {oys6.min()}-{oys6.max()}")

# =============================================================================
# 生成标注分析图
# =============================================================================
print("\n" + "=" * 60)
print("GENERATING ANNOTATED ANALYSIS IMAGES")
print("=" * 60)

for fname, title in [('01_initial.png', 'Initial State'), 
                      ('04_box3_added.png', '3 Boxes Added'),
                      ('05_after_execute.png', 'After Execution'),
                      ('07_top_view.png', 'Top View')]:
    img = Image.open(os.path.join(img_dir, fname))
    draw = ImageDraw.Draw(img)
    
    # 画网格线帮助定位
    for x_pct in [0.10, 0.20, 0.30, 0.40, 0.50, 0.62]:
        x = int(w * x_pct)
        draw.line([(x, 0), (x, h)], fill=(100, 100, 100, 100), width=1)
        draw.text((x+2, 5), f"{x_pct:.0%}", fill=(80, 80, 80))
    for y_pct in [0.10, 0.20, 0.30, 0.40, 0.50, 0.60, 0.70, 0.80, 0.90]:
        y = int(h * y_pct)
        draw.line([(0, y), (w, y)], fill=(100, 100, 100, 100), width=1)
    
    # 标注面板区域
    draw.rectangle([int(w*0.635), int(h*0.68), w-2, int(h*0.99)], outline='blue', width=3)
    draw.text((int(w*0.64), int(h*0.68)+5), "Scene Editor", fill='blue')
    
    draw.rectangle([int(w*0.635), int(h*0.34), w-2, int(h*0.67)], outline='green', width=3)
    draw.text((int(w*0.64), int(h*0.34)+5), "Robot Control", fill='green')
    
    draw.rectangle([int(w*0.635), int(h*0.04), w-2, int(h*0.33)], outline='red', width=3)
    draw.text((int(w*0.64), int(h*0.04)+5), "Task Control", fill='red')
    
    draw.rectangle([int(w*0.01), int(h*0.97), int(w*0.99), h-2], outline='orange', width=3)
    
    out_path = os.path.join(out_dir, f'annotated_{fname}')
    img.save(out_path)
    print(f"  Saved {out_path}")

# 生成对比拼图: 初始 vs 执行后
print("\n  Creating comparison strip...")
imgs = []
for f in ['01_initial.png', '04_box3_added.png', '05_after_execute.png', '07_top_view.png']:
    im = Image.open(os.path.join(img_dir, f))
    im_small = im.resize((w//3, h//3), Image.LANCZOS)
    imgs.append(im_small)

strip = Image.new('RGB', (w//3 * 2, h//3 * 2), (255, 255, 255))
for i, im in enumerate(imgs):
    x = (i % 2) * (w // 3)
    y = (i // 2) * (h // 3)
    strip.paste(im, (x, y))
strip.save(os.path.join(out_dir, 'comparison_2x2.png'))
print("  Saved comparison_2x2.png")

# =============================================================================
# 提取面板区域进行放大分析
# =============================================================================
print("\n" + "=" * 60)
print("EXTRACTING PANEL CLOSE-UPS")
print("=" * 60)

img_exec = Image.open(os.path.join(img_dir, '05_after_execute.png'))

# 提取场景编辑面板
scene_panel = img_exec.crop((int(w*0.635), int(h*0.01), w, int(h*0.32)))
scene_panel.save(os.path.join(out_dir, 'closeup_scene_editor.png'))
print(f"  Scene Editor panel: {scene_panel.size}")

# 提取机器人控制面板
rob_panel = img_exec.crop((int(w*0.635), int(h*0.33), w, int(h*0.66)))
rob_panel.save(os.path.join(out_dir, 'closeup_robot_ctrl.png'))
print(f"  Robot Control panel: {rob_panel.size}")

# 提取任务控制面板
task_panel = img_exec.crop((int(w*0.635), int(h*0.67), w, int(h*0.96)))
task_panel.save(os.path.join(out_dir, 'closeup_task_ctrl.png'))
print(f"  Task Control panel: {task_panel.size}")

# 提取状态栏
status_bar = img_exec.crop((0, int(h*0.97), w, h))
status_bar.save(os.path.join(out_dir, 'closeup_status_bar.png'))
print(f"  Status bar: {status_bar.size}")

# 提取3D场景的关键区域
# 传送带区域 (右上方)
conv_region = img_exec.crop((int(w*0.30), int(h*0.10), int(w*0.55), int(h*0.60)))
conv_region.save(os.path.join(out_dir, 'closeup_conveyor.png'))
print(f"  Conveyor region: {conv_region.size}")

# 码垛区域 (中间偏左)
pallet_region = img_exec.crop((int(w*0.05), int(h*0.10), int(w*0.35), int(h*0.65)))
pallet_region.save(os.path.join(out_dir, 'closeup_pallet.png'))
print(f"  Pallet region: {pallet_region.size}")

# =============================================================================
# 放大分析面板文字可读性
# =============================================================================
print("\n" + "=" * 60)
print("CHECKING PANEL TEXT READABILITY")
print("=" * 60)

# 检查Scene Editor面板中是否有按钮/文字可见
se_arr = np.array(scene_panel)
# 深色文字像素 (黑色/深蓝)
dark_text = (se_arr[:,:,0] < 60) & (se_arr[:,:,1] < 60) & (se_arr[:,:,2] < 60)
white_bg = (se_arr[:,:,0] > 230) & (se_arr[:,:,1] > 230) & (se_arr[:,:,2] > 230)
green_btn = (se_arr[:,:,1] > 130) & (se_arr[:,:,0] < 100)
red_btn = (se_arr[:,:,0] > 150) & (se_arr[:,:,1] < 80) & (se_arr[:,:,2] < 80)
print(f"  Scene Editor: dark_text={dark_text.sum()}, white_bg={white_bg.sum()}, green_btn={green_btn.sum()}, red_btn={red_btn.sum()}")

# 检查Task Control面板
tc_arr = np.array(task_panel)
dark_text_tc = (tc_arr[:,:,0] < 60) & (tc_arr[:,:,1] < 60) & (tc_arr[:,:,2] < 60)
blue_btn = (tc_arr[:,:,2] > 130) & (tc_arr[:,:,0] < 80)
green_btn_tc = (tc_arr[:,:,1] > 130) & (tc_arr[:,:,0] < 100)
red_btn_tc = (tc_arr[:,:,0] > 150) & (tc_arr[:,:,1] < 80) & (tc_arr[:,:,2] < 80)
print(f"  Task Control: dark_text={dark_text_tc.sum()}, blue_btn={blue_btn.sum()}, green_btn={green_btn_tc.sum()}, red_btn={red_btn_tc.sum()}")

# =============================================================================
# 最终问题汇总
# =============================================================================
print("\n" + "=" * 60)
print("ISSUE SUMMARY")
print("=" * 60)

# 基于分析自动推断问题
# 1) Place标记检查 - 初始时是否可见
if red_count > 500:
    add_issue('LOW', 'UI', 'Place marker (red triangle) visible at startup - might confuse users')

# 2) 初始时Pick标记
if green_count > 100:
    add_issue('LOW', 'UI', 'Pick marker visible at startup without any boxes')

# 3) 检查尺寸比例
# 图像是2719x1484, 面板占35.5%宽度=965px, 3D占62%=1686px
scene_pct = 0.62
panel_pct = 0.355
print(f"\n  Layout: Scene={scene_pct*100:.0f}%, Panel={panel_pct*100:.0f}%, Gap={100-scene_pct*100-panel_pct*100:.1f}%")

# 4) 检查前后对比中场景变化量
diff_4_5 = np.abs(arr4.astype(int) - arr5.astype(int))
changed_px = (diff_4_5.max(axis=2) > 30).sum()
total_px = h * w
change_ratio = changed_px / total_px
print(f"\n  Change ratio (box3 -> after_exec): {change_ratio:.3f} ({changed_px} / {total_px} pixels)")

for issue in issues:
    pass

print(f"\nTotal issues found: {len(issues)}")
print("\nDone!")
