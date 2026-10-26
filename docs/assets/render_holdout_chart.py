"""Render documented aggregate errors from the 2026-10-27 remote native fit.

Source: outputs/acceptance/calibration.json -> fit.holdout_pixels/metres.
Only summary statistics are embedded; no input samples or dataset are stored.
Run from any directory: python docs/assets/render_holdout_chart.py
"""
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 12,
                     'axes.spines.top': False, 'axes.spines.right': False})
fig, axes = plt.subplots(1, 2, figsize=(12, 4.8))
fig.patch.set_facecolor('#f7f9fc')
series = [
    ('Pixel residual', 'pixels', [6.603374510541044, 30.722352148379002]),
    ('Ground-plane residual', 'metres', [0.5479776735278257, 1.7493618888401896]),
]
for ax, (title, units, values) in zip(axes, series):
    ax.set_facecolor('#f7f9fc')
    bars = ax.bar(['Median', '95th percentile'], values, color=['#347ac0', '#dd9b35'], width=.52)
    ax.set_title(title, fontsize=15, fontweight='bold', loc='left', pad=18)
    ax.set_ylabel(units)
    ax.set_ylim(0, max(values)*1.25)
    ax.grid(axis='y', color='#dce2eb', linewidth=.8)
    ax.set_axisbelow(True)
    for bar, value in zip(bars, values):
        ax.annotate(f'{value:.2f}', (bar.get_x()+bar.get_width()/2, value),
                    xytext=(0, 8), textcoords='offset points', ha='center', fontweight='bold')
fig.suptitle('ES-MEOT native calibration / temporal holdout', x=.08, ha='left',
             fontsize=19, fontweight='bold', color='#172033')
fig.text(.08, .87, '5,699 holdout pairs  |  110 unseen camera IDs  |  separate units per panel', color='#536176', fontsize=11)
fig.text(.08, .025, 'Automatic correspondence residuals; not surveyed extrinsic accuracy.', color='#536176', fontsize=10)
fig.subplots_adjust(top=.73, bottom=.16, left=.08, right=.97, wspace=.3)
fig.savefig(Path(__file__).resolve().parents[2] / 'assets' / 'esmeot_holdout_errors.png', dpi=160)
plt.close(fig)
