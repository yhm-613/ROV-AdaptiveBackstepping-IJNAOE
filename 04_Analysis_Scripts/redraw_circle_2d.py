# -*- coding: utf-8 -*-
"""
redraw_circle_2d.py —— 圆形轨迹图 2D 重绘（fig_3/4/5 = 论文 Fig.6/7/8，包内 fig_6/7/8.pdf）
形式：每控制器一张单栏图（90 mm），双面板
  (a) x-y 平面俯视：参考圆 vs 实际轨迹（贴圆质量一目了然，无 3D 透视畸变）
  (b) 深度 z(t)：参考深度 2 m vs 实际（下潜捕获段）
规范：图内无标题；灰度可辨（实线=实际，虚线=参考）；折算字号 8-9 pt；
      三张图坐标范围统一（±3.2 m / z 0-3.2 m / t 0-100 s），横向可比。
数据源（2026-10-14 起）：CEP投稿上传包 .../results_new_补强/RE_{PID,BS,ADP}_circle.mat
（R2024b 单环境重跑 = 论文 Table 5 口径；eout = eta，前 3 列 m）
独立脚本，不改动 06_分析脚本 既有脚本。
"""
import h5py, numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import os

BASE = r'D:/yanjiusheng/yhm论文/自适应反步/CEP投稿上传包_20261013/05_Supplementary_Data/01_仿真结果_补强/results_new_补强'
OUT  = os.path.dirname(os.path.abspath(__file__))
PRE  = os.path.join(OUT, 'preview')
os.makedirs(PRE, exist_ok=True)

R, OM, T_END = 2.0, 0.2, 100.0
LIM = 3.2                      # xy 统一范围，z 上限同
FS  = 8                        # 图内字号 = 纸上折算字号（图宽 90 mm = 栏宽）
LW_REF, LW_ACT = 0.8, 1.0
C_REF, C_ACT = '0.40', '0.05'  # 灰度：参考中灰虚线，实际近黑实线
DASH = (0, (5, 3))             # 参考线规整虚线段
DS = 4                         # 抽稀因子（10 kHz -> 2.5 kHz，矢量文件更小）

FIGS = [  # (输出名, mat 文件, 控制器标签)
    ('fig_3', 'RE_PID_circle.mat', 'PID'),
    ('fig_4', 'RE_BS_circle.mat',  'backstepping'),
    ('fig_5', 'RE_ADP_circle.mat', 'adaptive backstepping'),
]

plt.rcParams.update({
    'font.family': 'serif',
    'font.serif': ['Times New Roman', 'STIXGeneral', 'Times'],
    'mathtext.fontset': 'stix',
    'font.size': FS,
    'axes.linewidth': 0.6,
    'xtick.direction': 'in', 'ytick.direction': 'in',
    'xtick.major.size': 2.8, 'ytick.major.size': 2.8,
    'xtick.minor.size': 1.6, 'ytick.minor.size': 1.6,
    'xtick.major.pad': 1.8, 'ytick.major.pad': 1.8,
    'xtick.top': True, 'ytick.right': True,     # 四边框均有内向刻度
    'legend.frameon': False, 'legend.fontsize': FS - 1,
    'pdf.fonttype': 42,
})

MM = 1 / 25.4
FIG_W, FIG_H = 90, 61           # mm，单栏 90 mm
# 手工布局（mm -> figure 归一化）：两面板同高同底，(a) 严格正方形，图例同一水平线
AX_BOT, AX_H = 18.0, 40.0       # 轴框底/高
A1_X, A1_W = 11.0, 40.0         # (a) 正方形
A2_X, A2_W = 57.8, 28.7  # 右缘 86.5mm，给 100 刻度标签留 3.5mm         # (b)
LC_Y = 0.045                    # 图例基线（figure 坐标）

for out_name, mat_name, label in FIGS:
    with h5py.File(f'{BASE}/{mat_name}', 'r') as f:
        t = np.array(f['t']).ravel()
        eta = np.array(f['eout'][:3])
    x, y, z = eta[0], eta[1], eta[2]
    xd = R * np.cos(OM * t); yd = R * np.sin(OM * t); zd = np.full_like(t, 2.0)
    s = slice(None, None, DS)

    fig = plt.figure(figsize=(FIG_W * MM, FIG_H * MM))
    ax1 = fig.add_axes([A1_X / FIG_W, AX_BOT / FIG_H, A1_W / FIG_W, AX_H / FIG_H])
    ax2 = fig.add_axes([A2_X / FIG_W, AX_BOT / FIG_H, A2_W / FIG_W, AX_H / FIG_H])
    c1, c2 = (A1_X + A1_W / 2) / FIG_W, (A2_X + A2_W / 2) / FIG_W

    # ---- (a) x-y 平面：实际在下、参考圆在上层（虚线不被盖住）----
    ax1.plot(x[s], y[s], '-', color=C_ACT, lw=0.8, alpha=0.85,
             label='Actual', zorder=2)  # 细线+半透明：自交振荡段可分层看清
    ax1.plot(xd[s], yd[s], linestyle=DASH, color=C_REF, lw=LW_REF, label='Desired', zorder=3)
    ax1.plot([0], [0], 'o', ms=2.8, mfc='white', mec='k', mew=0.6, zorder=5)
    ax1.annotate('Start', xy=(0, 0), xytext=(0.22, -0.68), fontsize=FS - 2,
                 style='italic',
                 bbox=dict(fc='white', ec='none', pad=0.6, alpha=0.9))
    ax1.set_xlim(-LIM, LIM); ax1.set_ylim(-LIM, LIM)
    ax1.set_aspect('equal')
    ax1.set_xticks(range(-3, 4)); ax1.set_yticks(range(-3, 4))
    ax1.set_xlabel('$x$ (m)', labelpad=2.5); ax1.set_ylabel('$y$ (m)', labelpad=2.5)
    ax1.legend(loc='lower center', bbox_to_anchor=(c1, LC_Y), bbox_transform=fig.transFigure,
               ncol=2, handlelength=1.5, borderaxespad=0.0, handletextpad=0.45,
               columnspacing=1.2, fontsize=FS - 1)
    ax1.text(-0.02, 1.02, '(a)', transform=ax1.transAxes, fontsize=FS, fontweight='bold')

    # ---- (b) 深度 z(t) ----
    ax2.plot(t[s], z[s], '-', color=C_ACT, lw=0.9, alpha=0.9,
             label='Actual', zorder=2)
    ax2.plot(t[s], zd[s], color=C_REF, lw=1.3, label='Desired',
             dashes=(6, 3), zorder=4)  # 加宽置顶：重合段虚线清晰"骑"在实线上
    ax2.set_xlim(0, T_END); ax2.set_ylim(0, LIM)
    ax2.set_xticks(range(0, 101, 20)); ax2.set_yticks(range(0, 4))
    ax2.set_xlabel('$t$ (s)', labelpad=2.5); ax2.set_ylabel('$z$ (m)', labelpad=2.5)
    ax2.legend(loc='lower center', bbox_to_anchor=(c2, LC_Y), bbox_transform=fig.transFigure,
               ncol=2, handlelength=1.5, borderaxespad=0.0, handletextpad=0.45,
               columnspacing=1.2, fontsize=FS - 1)
    ax2.text(-0.02, 1.02, '(b)', transform=ax2.transAxes, fontsize=FS, fontweight='bold')

    for ax in (ax1, ax2):
        ax.minorticks_on()
        ax.tick_params(which='minor', length=1.6)

    for ext, dpi in [('png', 300), ('pdf', None)]:
        p = os.path.join(PRE, f'{out_name}.{ext}')
        fig.savefig(p, dpi=dpi) if dpi else fig.savefig(p)
        print('saved', p)
    plt.close(fig)

print('DONE')
