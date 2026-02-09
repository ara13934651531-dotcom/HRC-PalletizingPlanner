"""细致分析面板布局、按钮排列、标签遮挡等视觉问题"""
import os
import numpy as np
from PIL import Image

img_dir = '/home/ara/桌面/X86_test/ArmCollisionModel/pic/sim_test'
out_dir = os.path.join(img_dir, 'analysis')

issues = []
def issue(sev, cat, msg):
    issues.append((sev, cat, msg))
    print(f"  !!! [{sev}] {cat}: {msg}")

# ====== 分析05 (执行后) 的面板细节 ======
print("="*70)
print("PANEL DETAIL ANALYSIS - 05_after_execute.png")
print("="*70)

img = Image.open(os.path.join(img_dir, '05_after_execute.png'))
arr = np.array(img)
h, w = arr.shape[:2]

# ---------- Scene Editor Panel (top-right, ~68%-99% of figure height) ----------
# Figure height 0.68-0.99 means y in image: (1-0.99)*h to (1-0.68)*h = 0.01h to 0.32h
se = arr[int(h*0.01):int(h*0.32), int(w*0.635):, :]
se_h, se_w = se.shape[:2]
print(f"\nScene Editor region: {se_w}x{se_h}")

# 检查白色输入框 (edit boxes) — 应该有几个
white_mask = (se[:,:,0] > 240) & (se[:,:,1] > 240) & (se[:,:,2] > 240)
white_rows = white_mask.sum(axis=1)
# 白色区域分布在哪几行
white_bands = []
in_band = False
band_start = 0
for i, cnt in enumerate(white_rows):
    if cnt > se_w * 0.02 and not in_band:
        in_band = True; band_start = i
    elif cnt < se_w * 0.01 and in_band:
        in_band = False; white_bands.append((band_start, i, i-band_start))
if in_band: white_bands.append((band_start, se_h, se_h-band_start))
print(f"  White bands (edit boxes + bg): {len(white_bands)}")
for b in white_bands:
    print(f"    y={b[0]}-{b[1]}, height={b[2]}px")

# 检查绿色Add按钮
green_btn = (se[:,:,1] > 130) & (se[:,:,0] < 120) & (se[:,:,2] < 120)
if green_btn.sum() > 100:
    gys, gxs = np.where(green_btn)
    print(f"  Green 'Add' button: y={gys.min()}-{gys.max()}, x={gxs.min()}-{gxs.max()}")
else:
    issue('HIGH', 'Layout', 'Add button not visible in Scene Editor')

# 红色Del按钮
red_btn = (se[:,:,0] > 170) & (se[:,:,1] < 80) & (se[:,:,2] < 80)
if red_btn.sum() > 100:
    rys, rxs = np.where(red_btn)
    print(f"  Red 'Del' button: y={rys.min()}-{rys.max()}, x={rxs.min()}-{rxs.max()}")
else:
    issue('MEDIUM', 'Layout', 'Del button not visible')

# 蓝色listbox边框
# Listbox typically has a light background with dark border

# ---------- Robot Control Panel (middle-right) ----------
# Figure 0.34-0.67 → image y: (1-0.67)*h to (1-0.34)*h = 0.33h to 0.66h
rc = arr[int(h*0.33):int(h*0.66), int(w*0.635):, :]
rc_h, rc_w = rc.shape[:2]
print(f"\nRobot Control region: {rc_w}x{rc_h}")

# 检查滑块 — 应该有6个灰色/蓝色横条
# 滑块通常是水平灰色条
slider_gray = (rc[:,:,0] > 180) & (rc[:,:,0] < 220) & \
              (np.abs(rc[:,:,0].astype(int) - rc[:,:,1].astype(int)) < 10) & \
              (np.abs(rc[:,:,1].astype(int) - rc[:,:,2].astype(int)) < 10)
slider_rows = slider_gray.sum(axis=1)
slider_bands = []
in_band = False
for i, cnt in enumerate(slider_rows):
    if cnt > rc_w * 0.05 and not in_band:
        in_band = True; band_start = i
    elif cnt < rc_w * 0.03 and in_band:
        in_band = False; slider_bands.append((band_start, i, i-band_start))
if in_band: slider_bands.append((band_start, rc_h, rc_h-band_start))
print(f"  Slider-like gray bands: {len(slider_bands)}")
if len(slider_bands) < 6:
    issue('MEDIUM', 'Layout', f'Expected 6 slider bands, found {len(slider_bands)}')

# 检查Home/Zero按钮 (蓝色/灰色)
blue_rc = (rc[:,:,2] > 150) & (rc[:,:,0] < 100) & (rc[:,:,1] < 130)
if blue_rc.sum() > 100:
    bys, bxs = np.where(blue_rc)
    print(f"  Blue button (Home): y={bys.min()}-{bys.max()}, x={bxs.min()}-{bxs.max()}")

# ---------- Task Control Panel (bottom-right) ----------
# Figure 0.04-0.33 → image y: (1-0.33)*h to (1-0.04)*h = 0.67h to 0.96h
tc = arr[int(h*0.67):int(h*0.96), int(w*0.635):, :]
tc_h, tc_w = tc.shape[:2]
print(f"\nTask Control region: {tc_w}x{tc_h}")

# 检查各种按钮
blue_tc = (tc[:,:,2] > 130) & (tc[:,:,0] < 80)
green_tc = (tc[:,:,1] > 130) & (tc[:,:,0] < 100) & (tc[:,:,2] < 100)
red_tc = (tc[:,:,0] > 160) & (tc[:,:,1] < 80) & (tc[:,:,2] < 80)
orange_tc = (tc[:,:,0] > 160) & (tc[:,:,1] > 80) & (tc[:,:,1] < 140) & (tc[:,:,2] < 80)
print(f"  Blue buttons: {blue_tc.sum()} px")
print(f"  Green buttons: {green_tc.sum()} px")
print(f"  Red buttons: {red_tc.sum()} px")
print(f"  Orange/brown buttons: {orange_tc.sum()} px")

# 检查白色编辑框
white_tc = (tc[:,:,0] > 240) & (tc[:,:,1] > 240) & (tc[:,:,2] > 240)
print(f"  White edit areas: {white_tc.sum()} px")

# 横向检查各行内容分布
row_nonbg = []
bg_color = np.array([245, 245, 250])  # approximate panel bg
for y in range(0, tc_h, tc_h//20):
    row = tc[y, :, :]
    non_bg = np.abs(row.astype(int) - bg_color).max(axis=1) > 20
    row_nonbg.append((y, non_bg.sum()))

print(f"  Row content density (y, non-bg-pixels):")
empty_rows = 0
for y, cnt in row_nonbg:
    if cnt < 10:
        empty_rows += 1
    #print(f"    y={y}: {cnt}")
print(f"  Empty rows: {empty_rows}/{len(row_nonbg)}")
if empty_rows > len(row_nonbg) * 0.4:
    issue('MEDIUM', 'Layout', 'Task Control panel has too much empty space')

# ---------- Status Bar ----------
sb = arr[int(h*0.97):, :, :]
sb_h, sb_w = sb.shape[:2]
print(f"\nStatus bar region: {sb_w}x{sb_h}")
# 应该有蓝色背景+白色文字
blue_bg = (sb[:,:,2] > 100) & (sb[:,:,0] < 80) & (sb[:,:,1] < 120)
white_text = (sb[:,:,0] > 220) & (sb[:,:,1] > 220) & (sb[:,:,2] > 220)
print(f"  Blue background: {blue_bg.sum()} px")
print(f"  White text: {white_text.sum()} px")
if white_text.sum() < 100:
    issue('MEDIUM', 'UI', 'Status bar text may not be readable')

# ====== 3D场景分析 ======
print("\n" + "="*70)
print("3D SCENE ANALYSIS")
print("="*70)

scene = arr[int(h*0.03):int(h*0.95), int(w*0.01):int(w*0.62), :]
sh, sw = scene.shape[:2]

# 分析颜色占比
total_px = sh * sw
light_bg = (scene[:,:,0] > 220) & (scene[:,:,1] > 220) & (scene[:,:,2] > 220)
light_pct = light_bg.sum() / total_px * 100
print(f"\n  Light/white background: {light_pct:.1f}% of scene")
if light_pct > 50:
    issue('MEDIUM', 'Rendering', f'Scene is {light_pct:.0f}% blank/white - too much empty space')

# 检查地面 (浅灰色)
ground = (scene[:,:,0] > 220) & (scene[:,:,0] < 245) & \
         (scene[:,:,1] > 220) & (scene[:,:,1] < 245) & \
         (scene[:,:,2] > 215) & (scene[:,:,2] < 240)
ground_pct = ground.sum() / total_px * 100
print(f"  Ground plane: {ground_pct:.1f}%")

# 框架/码垛蓝色
pallet_blue = (scene[:,:,2] > 100) & (scene[:,:,2] < 180) & \
              (scene[:,:,0] < 80) & (scene[:,:,1] < 120)
pallet_pct = pallet_blue.sum() / total_px * 100
print(f"  Pallet/frame blue: {pallet_pct:.1f}%")

# 传送带灰色
conv_gray = (scene[:,:,0] > 60) & (scene[:,:,0] < 110) & \
            (np.abs(scene[:,:,0].astype(int) - scene[:,:,1].astype(int)) < 15) & \
            (np.abs(scene[:,:,1].astype(int) - scene[:,:,2].astype(int)) < 15)
conv_pct = conv_gray.sum() / total_px * 100
print(f"  Conveyor gray: {conv_pct:.1f}%")

# 机器人颜色
capsule_colors = (scene[:,:,2] > 140) & (scene[:,:,2] < 230) & \
                 (scene[:,:,0] > 100) & (scene[:,:,0] < 220) & \
                 (scene[:,:,1] > 100) & (scene[:,:,1] < 220) & \
                 ~light_bg & ~ground
capsule_pct = capsule_colors.sum() / total_px * 100
print(f"  Mid-tone (capsule/objects): {capsule_pct:.1f}%")

# 检查柜子是否可见 (浅黄/奶油色)
cabinet = (scene[:,:,0] > 230) & (scene[:,:,1] > 230) & \
          (scene[:,:,2] > 220) & (scene[:,:,2] < 242) & \
          (scene[:,:,0] > scene[:,:,2] + 3)
cab_pct = cabinet.sum() / total_px * 100
print(f"  Cabinet cream: {cab_pct:.1f}%")

# ======= 对比初始和执行后 =======
print("\n" + "="*70)
print("COMPARING STATES")
print("="*70)

img_init = np.array(Image.open(os.path.join(img_dir, '01_initial.png')))
img_3box = np.array(Image.open(os.path.join(img_dir, '04_box3_added.png')))
img_exec = np.array(Image.open(os.path.join(img_dir, '05_after_execute.png')))

# 初始→3箱 差异
diff_1_4 = np.abs(img_init.astype(int) - img_3box.astype(int)).max(axis=2)
changed_1_4 = (diff_1_4 > 30).sum()
print(f"  Init → 3-box change: {changed_1_4} px ({changed_1_4/total_px*100:.2f}%)")

# 3箱→执行后 差异
diff_4_5 = np.abs(img_3box.astype(int) - img_exec.astype(int)).max(axis=2)
changed_4_5 = (diff_4_5 > 30).sum()
print(f"  3-box → post-exec change: {changed_4_5} px ({changed_4_5/total_px*100:.2f}%)")

# 找到差异最大的区域
if changed_4_5 > 0:
    diff_scene = diff_4_5[int(h*0.03):int(h*0.95), int(w*0.01):int(w*0.62)]
    diff_panel = diff_4_5[int(h*0.03):int(h*0.95), int(w*0.635):]
    print(f"    Scene area changes: {(diff_scene>30).sum()}")
    print(f"    Panel area changes: {(diff_panel>30).sum()}")
    
    # 差异区域的Y分布
    diff_y = diff_scene.max(axis=1)
    active_ys = np.where(diff_y > 50)[0]
    if len(active_ys) > 0:
        print(f"    Scene diff Y range: {active_ys.min()}-{active_ys.max()} (of {sh})")

# ======= 俯视图分析 (空间布局) =======
print("\n" + "="*70)
print("TOP VIEW SPATIAL ANALYSIS")
print("="*70)
img_top = np.array(Image.open(os.path.join(img_dir, '07_top_view.png')))
scene_top = img_top[int(h*0.03):int(h*0.95), int(w*0.01):int(w*0.62), :]

# 统计俯视图中各区域占比
light_top = (scene_top[:,:,0] > 220) & (scene_top[:,:,1] > 220) & (scene_top[:,:,2] > 220)
blue_top = (scene_top[:,:,2] > 100) & (scene_top[:,:,2] < 180) & (scene_top[:,:,0] < 80)
gray_top = (scene_top[:,:,0] > 60) & (scene_top[:,:,0] < 110) & \
           (np.abs(scene_top[:,:,0].astype(int) - scene_top[:,:,1].astype(int)) < 15)
orange_top = (scene_top[:,:,0] > 200) & (scene_top[:,:,1] > 50) & (scene_top[:,:,1] < 140) & (scene_top[:,:,2] < 60)

tpx = scene_top.shape[0] * scene_top.shape[1]
print(f"  Light/empty: {light_top.sum()/tpx*100:.1f}%")
print(f"  Blue (pallet/frame): {blue_top.sum()/tpx*100:.1f}%")
print(f"  Gray (conveyor): {gray_top.sum()/tpx*100:.1f}%")
print(f"  Orange (trail): {orange_top.sum()/tpx*100:.1f}%")

# 检查轨迹是否合理地从传送带到码垛区
if orange_top.sum() > 500:
    oys, oxs = np.where(orange_top)
    # 质心
    cx, cy = oxs.mean(), oys.mean()
    print(f"  Trail centroid: ({cx:.0f}, {cy:.0f}) of ({sw}x{sh})")
    # 方差
    print(f"  Trail X spread: std={oxs.std():.0f}")
    print(f"  Trail Y spread: std={oys.std():.0f}")

# ====== 最终问题汇总 ======
print("\n" + "="*70)
print("ALL IDENTIFIED ISSUES")
print("="*70)

# 追加基于代码审查的问题
issue('HIGH', 'UX', 'Place marker (red ▼) appears at startup before user sets targets - autoFillPlace() called in init')
issue('HIGH', 'UX', 'When Place marker set at init, its position may be invisible from default 135° view angle')
issue('MEDIUM', 'Layout', 'Scene occupies 62% width but visual content is small in center - consider tighter axis limits')
issue('MEDIUM', 'Logic', 'convBoxCount tracks added boxes but does not decrement when Del is pressed')
issue('MEDIUM', 'Logic', 'After deleting a box, Pick/Place targets still point to deleted object')
issue('MEDIUM', 'Rendering', 'CAPSULE_ALPHA=0.40 makes robot very transparent, hard to see against light background')
issue('LOW', 'UX', 'Speed slider range 3-40 — min=3 steps may be too fast for complex motions')
issue('LOW', 'UX', 'No visual feedback for carrying box during animation (no box attached to TCP)')
issue('MEDIUM', 'Logic', 'Reset Count resets convBoxCount but does not reset box names or positions already in scene')
issue('LOW', 'Layout', 'Status bar height is only ~3% of figure - text may be hard to read')
issue('MEDIUM', 'Logic', 'findPickBox uses norm(pos-pickTarget) but pickTarget Z includes box half-height while pos is center')
issue('LOW', 'Layout', 'Object listbox is small relative to panel, may truncate long names')
issue('MEDIUM', 'Logic', 'Pallet position uses frameCY-pallet.dy/2+0.02 as start, but frame/pallet edge may not align exactly')
issue('HIGH', 'Logic', 'executeCB never removes the picked box from sceneObjs array - hidden but still tracked')
issue('MEDIUM', 'Rendering', 'Grid lines obscure scene details at certain angles')
issue('LOW', 'UX', 'No way to re-show a hidden (picked) box without clearing scene')
issue('LOW', 'Rendering', 'Conveyor rollers rendered with CYL_N=8 faces, looks octagonal not round')
issue('MEDIUM', 'UX', 'After full pallet (6 boxes), nextPalletPos keeps incrementing to invalid layer 3+')

for i, (sev, cat, msg) in enumerate(issues, 1):
    print(f"  {i:2d}. [{sev:6s}] [{cat:10s}] {msg}")

print(f"\nTotal: {len(issues)} issues")
print(f"  HIGH: {sum(1 for s,_,_ in issues if s=='HIGH')}")
print(f"  MEDIUM: {sum(1 for s,_,_ in issues if s=='MEDIUM')}")
print(f"  LOW: {sum(1 for s,_,_ in issues if s=='LOW')}")
