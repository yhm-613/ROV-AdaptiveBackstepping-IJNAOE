# -*- coding: utf-8 -*-
"""
make_graphical_abstract.py —— CEP Graphical Abstract（Elsevier 规格）
规格：Elsevier GA 最小 531 x 1328 px (h x w)，推荐 2 倍。
本脚本按 300 dpi 输出 13.28 x 5.31 in -> 3984 x 1593 px（超出 2x 规格），
缩印到 5 x 13 cm 时图内最小字号约 6-7 pt，可读。
三栏：左=方法示意框图；中=真实数据结果（圆轨迹叠图 + RMS 条形图）；
      右=HIL 板卡照片 + 要点。
数据源（2026-10-14 起）：CEP投稿上传包 .../results_new_补强/RE_{PID,ADP}_circle.mat（论文口径）
照片：02_投稿材料/...-1013/figures_src/board_photo_YS-F4Pro.jpg
独立脚本，不改动其他脚本。
"""
import os
import h5py
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch

BASE = r'D:/yanjiusheng/yhm论文/自适应反步'
DATA = os.path.join(BASE, 'CEP投稿上传包_20261013/05_Supplementary_Data/01_仿真结果_补强/results_new_补强')
PHOTO = os.path.join(BASE, '02_投稿材料/Manuscript_Yang_AdaptiveBackstepping-6DOF-ROV - 1013/figures_src/board_photo_YS-F4Pro.jpg')
OUT = os.path.join(BASE, 'CEP投稿上传包_20261013/06_Graphical_Abstract')
os.makedirs(OUT, exist_ok=True)

# ---- 配色 ----
C_ADP = '#1f4e79'   # 深蓝 = 自适应反步（强调）
C_PID = '#9bb7d4'   # 浅蓝灰 = PID（弱化）
C_BSF = '#5b87b0'   # 中蓝 = 反步
C_REF = '0.25'      # 参考虚线
C_BOX = '#eaf1f8'   # 框图底色
C_EDGE = '#1f4e79'

plt.rcParams.update({
    'font.family': 'sans-serif',
    'font.sans-serif': ['Arial', 'Helvetica', 'DejaVu Sans'],
    'mathtext.fontset': 'stix',
})

FW, FH = 13.28, 5.31          # inch，Elsevier GA 比例（1328x531 px 的 2x+）
fig = plt.figure(figsize=(FW, FH))
fig.patch.set_facecolor('white')

# ================= 栏标题 =================
fig.text(0.15, 0.955, 'Grouped adaptive backstepping',
         ha='center', va='top', fontsize=18, fontweight='bold', color=C_ADP)
fig.text(0.50, 0.955, 'Circular trajectory tracking',
         ha='center', va='top', fontsize=18, fontweight='bold', color=C_ADP)
fig.text(0.85, 0.955, 'Hardware-in-the-loop validation',
         ha='center', va='top', fontsize=18, fontweight='bold', color=C_ADP)

# ================= 左栏：方法框图 =================
axm = fig.add_axes([0.015, 0.06, 0.26, 0.80])
axm.set_xlim(0, 10); axm.set_ylim(0, 10); axm.axis('off')

def box(x, y, w, h, lines, fs=14.5, fc=C_BOX, ec=C_EDGE, lw=1.6, bold_first=True):
    p = FancyBboxPatch((x, y), w, h, boxstyle='round,pad=0.12,rounding_size=0.25',
                       fc=fc, ec=ec, lw=lw, zorder=2)
    axm.add_patch(p)
    n = len(lines)
    for i, ln in enumerate(lines):
        yy = y + h - (i + 0.5) * h / n
        axm.text(x + w / 2, yy, ln, ha='center', va='center', fontsize=fs,
                 fontweight='bold' if (bold_first and i == 0) else 'normal',
                 color='black', zorder=3)

def arrow(x1, y1, x2, y2, style='-|>', lw=2.2, color=C_ADP, conn='arc3,rad=0'):
    a = FancyArrowPatch((x1, y1), (x2, y2), arrowstyle=style, lw=lw,
                        color=color, mutation_scale=22, zorder=1, connectionstyle=conn)
    axm.add_patch(a)

# 主链：期望 -> 控制律 -> ROV -> 反馈
box(1.0, 7.9, 8.0, 1.6, [r'Desired trajectory  $\eta_d$'], fs=16)
box(1.0, 5.2, 8.0, 2.0, ['Backstepping law  ' + r'$\tau$',
                         'grouped 6-D adaptive gains',
                         r'$\hat{c}_1,\ \hat{c}_2$  + $\sigma$-mod.'], fs=14)
box(1.0, 2.5, 8.0, 1.6, ['6-DOF ROV dynamics', r'$\eta$ = position / attitude'], fs=14.5)
arrow(5.0, 7.9, 5.0, 7.25)
arrow(5.0, 5.2, 5.0, 4.15)
# 反馈环
arrow(9.1, 3.3, 9.9, 3.3, style='-', lw=2.0)
arrow(9.9, 3.3, 9.9, 6.2, style='-', lw=2.0)
arrow(9.9, 6.2, 9.05, 6.2)
axm.text(9.45, 4.7, r'$\eta$', fontsize=16, style='italic', ha='center', color=C_ADP)
# 注释
axm.text(5.0, 1.45, 'Gains initialised at the nominal values;',
         ha='center', fontsize=11.5, color='0.25')
axm.text(5.0, 0.62, 'adapt only as errors reveal uncertainty',
         ha='center', fontsize=11.5, color='0.25')

# ================= 中栏：真实数据 =================
with h5py.File(os.path.join(DATA, 'RE_PID_circle.mat'), 'r') as f:
    t = np.array(f['t']).ravel()
    e = np.array(f['eout'][:3])
x_pid, y_pid = e[0], e[1]
with h5py.File(os.path.join(DATA, 'RE_ADP_circle.mat'), 'r') as f:
    e = np.array(f['eout'][:3])
x_adp, y_adp = e[0], e[1]
R, OM = 2.0, 0.2
s = slice(None, None, 4)

axt = fig.add_axes([0.315, 0.12, 0.185, 0.66])
axt.set_aspect('equal')
axt.plot(x_pid[s], y_pid[s], '-', color=C_PID, lw=1.6, alpha=0.9, label='PID', zorder=2)
axt.plot(x_adp[s], y_adp[s], '-', color=C_ADP, lw=2.0, label='Adaptive BS', zorder=3)
th = np.linspace(0, 2 * np.pi, 200)
axt.plot(R * np.cos(th), R * np.sin(th), '--', color=C_REF, lw=1.3, label='Desired', zorder=4)
axt.set_xlim(-3.2, 3.2); axt.set_ylim(-3.2, 3.2)
axt.set_xticks([]); axt.set_yticks([])
for sp in axt.spines.values():
    sp.set_linewidth(0.8)
axt.legend(loc='upper center', bbox_to_anchor=(0.5, 1.035), ncol=3, frameon=False,
           fontsize=10.5, handlelength=1.2, columnspacing=0.7, handletextpad=0.35)

# RMS 条形图
axr = fig.add_axes([0.578, 0.21, 0.112, 0.54])
names = ['PID', 'BS', 'Adaptive\nBS']
vals = [2.6664, 0.7862, 0.4764]
cols = [C_PID, C_BSF, C_ADP]
yp = range(3)[::-1]
axr.barh(list(yp), vals, height=0.62, color=cols, edgecolor='none')
for i, v in zip(yp, vals):
    axr.text(v + 0.06, i, f'{v:.2f}', va='center', fontsize=14, color='black')
axr.set_yticks(list(yp)); axr.set_yticklabels(names, fontsize=12.5)
axr.set_xlim(0, 3.45)
axr.set_xticks([0, 1, 2, 3])
axr.tick_params(axis='x', labelsize=12)
axr.set_xlabel('Tracking RMS (m)', fontsize=13.5)
for sp in ('top', 'right'):
    axr.spines[sp].set_visible(False)
for sp in ('left', 'bottom'):
    axr.spines[sp].set_linewidth(0.8)
axr.text(0.4764 + 0.14, 0.5, '$-$82% (PID)\n$-$39% (BS)', va='center',
         fontsize=12, color=C_ADP, fontweight='bold', linespacing=1.4)
# 面板字母
axt.text(-0.05, 1.03, '(a)', transform=axt.transAxes, fontsize=16, fontweight='bold')
axr.text(-0.32, 1.03, '(b)', transform=axr.transAxes, fontsize=16, fontweight='bold')

# ================= 右栏：HIL 照片 =================
axp = fig.add_axes([0.700, 0.44, 0.295, 0.40])
axp.imshow(plt.imread(PHOTO))
axp.axis('off')
axp.text(0.5, -0.055, 'STM32F407 (ARM Cortex-M4F, 168 MHz),',
         transform=axp.transAxes, ha='center', va='top', fontsize=12, color='0.15')
axp.text(0.5, -0.135, 'running the generated C code',
         transform=axp.transAxes, ha='center', va='top', fontsize=12, color='0.15')

bullets = [
    ('Fixed 10 ms control period', 0.315),
    (r'$\leq$ 11.2% CPU per control step', 0.23),
    ('RMS within 0.8% of the PC run', 0.145),
]
for txt, yy in bullets:
    fig.text(0.715, yy, '\u25aa', fontsize=13, color=C_ADP, va='center')
    fig.text(0.733, yy, txt, fontsize=14, va='center', color='black')

# ================= 底部 takeaway =================
fig.text(0.5, 0.055,
         'Bounded grouped adaptive gains replace the two constant backstepping gains '
         r'— up to 82% lower tracking error than PID —',
         ha='center', va='bottom', fontsize=13, style='italic', color='0.15')
fig.text(0.5, 0.018,
         'and the same control law runs in real time on embedded hardware.',
         ha='center', va='bottom', fontsize=13, style='italic', color='0.15')

# ---- 输出 ----
out_png = os.path.join(OUT, 'graphical_abstract.png')
fig.savefig(out_png, dpi=300, facecolor='white')
from PIL import Image
im = Image.open(out_png)
print('saved:', out_png, im.size, f'{os.path.getsize(out_png)/1024:.0f} KB')
# 缩略预览
prev = os.path.join(OUT, '_preview.png')
im.thumbnail((1328, 531 * 3))
im.save(prev)
print('preview:', prev)
print('DONE')
