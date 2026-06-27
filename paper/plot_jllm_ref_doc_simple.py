"""
简化版可视化：只关注三个J-LLM在Ref-Doc上的对比
"""
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from scipy.stats import pearsonr
import sys
import io

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')

# 读取数据
df = pd.read_csv('paper/out/jllm_ref_doc_comparison.csv')

# 设置字体和样式
plt.rcParams['font.size'] = 11
plt.rcParams['font.family'] = 'sans-serif'
sns.set_palette("husl")

# 创建2x2布局
fig = plt.figure(figsize=(16, 12))

# 1. 相关性热力图
ax1 = plt.subplot(2, 2, 1)
jllm_cols = ['J-LLM-4.1-mini', 'J-LLM-4.1-nano', 'J-LLM-4o-mini']
jllm_data = df[jllm_cols]

# 计算Pearson相关矩阵
corr_matrix = jllm_data.corr(method='pearson')

# 绘制热力图
sns.heatmap(corr_matrix, annot=True, fmt='.4f', cmap='RdYlGn', 
            center=0.9, vmin=0.8, vmax=1.0, square=True, 
            linewidths=2, cbar_kws={'label': 'Pearson r'},
            ax=ax1, annot_kws={'size': 14, 'weight': 'bold'})
ax1.set_title('J-LLM Correlation (Ref-Doc)', fontsize=15, fontweight='bold', pad=15)
ax1.set_xticklabels(['4.1-mini', '4.1-nano', '4o-mini'], fontsize=12)
ax1.set_yticklabels(['4.1-mini', '4.1-nano', '4o-mini'], fontsize=12, rotation=0)

# 2. 散点图：4.1-mini vs 4o-mini
ax2 = plt.subplot(2, 2, 2)
x = df['J-LLM-4.1-mini']
y = df['J-LLM-4o-mini']

ax2.scatter(x, y, s=150, alpha=0.6, c='#FF6B6B', edgecolors='black', linewidths=1)

# 添加回归线
z = np.polyfit(x, y, 1)
p = np.poly1d(z)
x_line = np.linspace(x.min(), x.max(), 100)
ax2.plot(x_line, p(x_line), 'b-', linewidth=2.5, alpha=0.7, label='Linear fit')

# 添加对角线
min_val = min(x.min(), y.min())
max_val = max(x.max(), y.max())
ax2.plot([min_val, max_val], [min_val, max_val], 'k--', alpha=0.4, linewidth=2, label='y=x')

r, p_val = pearsonr(x, y)
ax2.text(0.05, 0.95, f'r = {r:.4f}\np < 0.001', 
         transform=ax2.transAxes, fontsize=13, verticalalignment='top',
         bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))

ax2.set_xlabel('GPT-4.1-mini', fontsize=13, fontweight='bold')
ax2.set_ylabel('GPT-4o-mini', fontsize=13, fontweight='bold')
ax2.set_title('4.1-mini vs 4o-mini', fontsize=15, fontweight='bold', pad=15)
ax2.legend(fontsize=11)
ax2.grid(True, alpha=0.3)

# 3. 散点图：4.1-mini vs 4.1-nano
ax3 = plt.subplot(2, 2, 3)
x = df['J-LLM-4.1-mini']
y = df['J-LLM-4.1-nano']

ax3.scatter(x, y, s=150, alpha=0.6, c='#4ECDC4', edgecolors='black', linewidths=1)

# 添加回归线
z = np.polyfit(x, y, 1)
p = np.poly1d(z)
x_line = np.linspace(x.min(), x.max(), 100)
ax3.plot(x_line, p(x_line), 'b-', linewidth=2.5, alpha=0.7, label='Linear fit')

# 添加对角线
min_val = min(x.min(), y.min())
max_val = max(x.max(), y.max())
ax3.plot([min_val, max_val], [min_val, max_val], 'k--', alpha=0.4, linewidth=2, label='y=x')

r, p_val = pearsonr(x, y)
ax3.text(0.05, 0.95, f'r = {r:.4f}\np < 0.001', 
         transform=ax3.transAxes, fontsize=13, verticalalignment='top',
         bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))

ax3.set_xlabel('GPT-4.1-mini', fontsize=13, fontweight='bold')
ax3.set_ylabel('GPT-4.1-nano', fontsize=13, fontweight='bold')
ax3.set_title('4.1-mini vs 4.1-nano', fontsize=15, fontweight='bold', pad=15)
ax3.legend(fontsize=11)
ax3.grid(True, alpha=0.3)

# 4. 箱线图：分数分布对比
ax4 = plt.subplot(2, 2, 4)
box_data = [df['J-LLM-4.1-mini'], df['J-LLM-4.1-nano'], df['J-LLM-4o-mini']]
labels = ['4.1-mini', '4.1-nano', '4o-mini']

bp = ax4.boxplot(box_data, labels=labels, patch_artist=True, showmeans=True,
                 meanprops=dict(marker='D', markerfacecolor='red', markersize=10, markeredgecolor='black'),
                 medianprops=dict(color='blue', linewidth=2.5),
                 boxprops=dict(facecolor='lightblue', alpha=0.7, linewidth=2),
                 whiskerprops=dict(linewidth=2),
                 capprops=dict(linewidth=2))

# 为每个箱线图添加统计信息
for i, (data, label) in enumerate(zip(box_data, labels)):
    mean_val = data.mean()
    median_val = data.median()
    ax4.text(i+1, mean_val + 1, f'μ={mean_val:.1f}', 
            ha='center', fontsize=10, fontweight='bold', color='red')

ax4.set_ylabel('Ref-Doc Score', fontsize=13, fontweight='bold')
ax4.set_title('Score Distribution Comparison', fontsize=15, fontweight='bold', pad=15)
ax4.grid(True, alpha=0.3, axis='y')
ax4.set_ylim([10, 50])

# 添加图例
from matplotlib.patches import Patch
legend_elements = [
    Patch(facecolor='lightblue', label='Box (IQR)'),
    plt.Line2D([0], [0], color='blue', linewidth=2.5, label='Median'),
    plt.Line2D([0], [0], marker='D', color='w', markerfacecolor='red', 
               markersize=8, label='Mean', markeredgecolor='black')
]
ax4.legend(handles=legend_elements, loc='upper right', fontsize=10)

plt.suptitle('J-LLM Evaluators Comparison (Ref-Doc Modality)', 
             fontsize=17, fontweight='bold', y=0.995)

plt.tight_layout(rect=[0, 0, 1, 0.99])

# 保存图表
output_png = 'paper/out/jllm_ref_doc_correlation.png'
plt.savefig(output_png, dpi=300, bbox_inches='tight', facecolor='white')
print(f"相关性分析图已保存到: {output_png}")

output_pdf = 'paper/out/jllm_ref_doc_correlation.pdf'
plt.savefig(output_pdf, format='pdf', bbox_inches='tight', facecolor='white')
print(f"PDF版本已保存到: {output_pdf}")

plt.close()

# 创建第二个图：Top 10模型的对比
fig2, ax = plt.subplots(figsize=(14, 8))

top10 = df.head(10)
x = np.arange(len(top10))
width = 0.25

colors = ['#FF6B6B', '#4ECDC4', '#95E1D3']
bars1 = ax.bar(x - width, top10['J-LLM-4.1-mini'], width, label='4.1-mini', 
               alpha=0.8, color=colors[0], edgecolor='black', linewidth=1.5)
bars2 = ax.bar(x, top10['J-LLM-4.1-nano'], width, label='4.1-nano', 
               alpha=0.8, color=colors[1], edgecolor='black', linewidth=1.5)
bars3 = ax.bar(x + width, top10['J-LLM-4o-mini'], width, label='4o-mini', 
               alpha=0.8, color=colors[2], edgecolor='black', linewidth=1.5)

# 添加数值标签
for bars in [bars1, bars2, bars3]:
    for bar in bars:
        height = bar.get_height()
        ax.text(bar.get_x() + bar.get_width()/2., height,
                f'{height:.1f}',
                ha='center', va='bottom', fontsize=9, fontweight='bold')

ax.set_xlabel('Model', fontsize=14, fontweight='bold')
ax.set_ylabel('Ref-Doc Score', fontsize=14, fontweight='bold')
ax.set_title('Top 10 Models: J-LLM Evaluators Comparison', fontsize=16, fontweight='bold', pad=20)
ax.set_xticks(x)
ax.set_xticklabels([m[:20] + '...' if len(m) > 20 else m for m in top10['Model']], 
                    rotation=45, ha='right', fontsize=10)
ax.legend(fontsize=12, loc='upper right')
ax.grid(True, alpha=0.3, axis='y')
ax.set_ylim([0, 50])

plt.tight_layout()

output_png2 = 'paper/out/jllm_ref_doc_top10.png'
plt.savefig(output_png2, dpi=300, bbox_inches='tight', facecolor='white')
print(f"Top 10对比图已保存到: {output_png2}")

output_pdf2 = 'paper/out/jllm_ref_doc_top10.pdf'
plt.savefig(output_pdf2, format='pdf', bbox_inches='tight', facecolor='white')
print(f"PDF版本已保存到: {output_pdf2}")

plt.close()

print("\n所有图表生成完成！")
