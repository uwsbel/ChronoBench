"""
可视化J-LLM评估器之间的相关性
"""
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from scipy.stats import pearsonr, spearmanr
import sys
import io

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')

# 读取数据
df = pd.read_csv('scoring/out/jllm_comparison.csv')

# 设置字体
plt.rcParams['font.size'] = 11
plt.rcParams['font.family'] = 'sans-serif'

# 创建图表 (2x2布局)
fig = plt.figure(figsize=(16, 14))

# 1. J-LLM评估器之间的Pearson相关性热力图
ax1 = plt.subplot(2, 2, 1)
jllm_cols = ['J-LLM-4.1-mini', 'J-LLM-4.1-nano', 'J-LLM-4o-mini']
jllm_data = df[jllm_cols]

# 计算Pearson相关矩阵
pearson_corr = jllm_data.corr(method='pearson')

# 绘制热力图
sns.heatmap(pearson_corr, annot=True, fmt='.4f', cmap='RdYlGn', center=0.9,
            vmin=0.8, vmax=1.0, square=True, linewidths=2, cbar_kws={'label': 'Pearson r'},
            ax=ax1, annot_kws={'size': 13, 'weight': 'bold'})
ax1.set_title('J-LLM评估器 - Pearson相关系数', fontsize=14, fontweight='bold', pad=15)
ax1.set_xticklabels(['4.1-mini', '4.1-nano', '4o-mini'], fontsize=12)
ax1.set_yticklabels(['4.1-mini', '4.1-nano', '4o-mini'], fontsize=12, rotation=0)

# 2. J-LLM评估器之间的Spearman相关性热力图
ax2 = plt.subplot(2, 2, 2)

# 计算Spearman相关矩阵
spearman_corr = jllm_data.corr(method='spearman')

sns.heatmap(spearman_corr, annot=True, fmt='.4f', cmap='RdYlGn', center=0.9,
            vmin=0.8, vmax=1.0, square=True, linewidths=2, cbar_kws={'label': 'Spearman ρ'},
            ax=ax2, annot_kws={'size': 13, 'weight': 'bold'})
ax2.set_title('J-LLM评估器 - Spearman相关系数 (秩相关)', fontsize=14, fontweight='bold', pad=15)
ax2.set_xticklabels(['4.1-mini', '4.1-nano', '4o-mini'], fontsize=12)
ax2.set_yticklabels(['4.1-mini', '4.1-nano', '4o-mini'], fontsize=12, rotation=0)

# 3. Modality之间的相关性热力图
ax3 = plt.subplot(2, 2, 3)
modality_cols = ['Ref-Doc', 'Ref', 'Doc']
modality_data = df[modality_cols]

modality_corr = modality_data.corr(method='pearson')

sns.heatmap(modality_corr, annot=True, fmt='.4f', cmap='Blues', center=0.85,
            vmin=0.7, vmax=1.0, square=True, linewidths=2, cbar_kws={'label': 'Pearson r'},
            ax=ax3, annot_kws={'size': 13, 'weight': 'bold'})
ax3.set_title('不同Modality - Pearson相关系数', fontsize=14, fontweight='bold', pad=15)
ax3.set_xticklabels(['Ref-Doc', 'Ref', 'Doc'], fontsize=12)
ax3.set_yticklabels(['Ref-Doc', 'Ref', 'Doc'], fontsize=12, rotation=0)

# 4. 散点图：比较不同J-LLM评估器
ax4 = plt.subplot(2, 2, 4)

# 4.1-mini vs 4.1-nano
ax4.scatter(df['J-LLM-4.1-mini'], df['J-LLM-4.1-nano'], 
           s=100, alpha=0.6, c='#FF6B6B', label='4.1-mini vs 4.1-nano', edgecolors='black', linewidths=0.5)

# 4.1-mini vs 4o-mini
ax4.scatter(df['J-LLM-4.1-mini'], df['J-LLM-4o-mini'], 
           s=100, alpha=0.6, c='#4ECDC4', label='4.1-mini vs 4o-mini', edgecolors='black', linewidths=0.5)

# 4.1-nano vs 4o-mini (需要调整x轴)
# 不绘制第三对，因为会使图表混乱

# 添加对角线
min_val = min(df[jllm_cols].min().min(), df[jllm_cols].min().min())
max_val = max(df[jllm_cols].max().max(), df[jllm_cols].max().max())
ax4.plot([min_val, max_val], [min_val, max_val], 'k--', alpha=0.3, linewidth=2, label='y=x')

# 添加回归线 (4.1-mini vs 4o-mini)
z = np.polyfit(df['J-LLM-4.1-mini'], df['J-LLM-4o-mini'], 1)
p = np.poly1d(z)
x_line = np.linspace(df['J-LLM-4.1-mini'].min(), df['J-LLM-4.1-mini'].max(), 100)
ax4.plot(x_line, p(x_line), 'b-', alpha=0.5, linewidth=2, label=f'拟合线 (r={pearson_corr.loc["J-LLM-4.1-mini", "J-LLM-4o-mini"]:.3f})')

ax4.set_xlabel('J-LLM-4.1-mini Score', fontsize=13, fontweight='bold')
ax4.set_ylabel('其他J-LLM Score', fontsize=13, fontweight='bold')
ax4.set_title('J-LLM评估器分数对比', fontsize=14, fontweight='bold', pad=15)
ax4.legend(fontsize=10, loc='upper left')
ax4.grid(True, alpha=0.3, linestyle='--')
ax4.set_xlim([10, 50])
ax4.set_ylim([10, 50])

# 调整布局
plt.tight_layout(pad=3.0)

# 保存图表
output_png = 'scoring/out/jllm_correlation_analysis.png'
plt.savefig(output_png, dpi=300, bbox_inches='tight', facecolor='white')
print(f"相关性分析图已保存到: {output_png}")

output_pdf = 'scoring/out/jllm_correlation_analysis.pdf'
plt.savefig(output_pdf, format='pdf', bbox_inches='tight', facecolor='white')
print(f"PDF版本已保存到: {output_pdf}")

plt.close()

# 创建第二个图表：分数分布对比
fig2, axes = plt.subplots(2, 2, figsize=(16, 12))

# 箱线图：J-LLM评估器分数分布
ax1 = axes[0, 0]
box_data = [df['J-LLM-4.1-mini'], df['J-LLM-4.1-nano'], df['J-LLM-4o-mini']]
bp = ax1.boxplot(box_data, labels=['4.1-mini', '4.1-nano', '4o-mini'],
                 patch_artist=True, showmeans=True,
                 boxprops=dict(facecolor='lightblue', alpha=0.7),
                 medianprops=dict(color='red', linewidth=2),
                 meanprops=dict(marker='D', markerfacecolor='green', markersize=8))
ax1.set_ylabel('Score', fontsize=13, fontweight='bold')
ax1.set_title('J-LLM评估器分数分布', fontsize=14, fontweight='bold', pad=15)
ax1.grid(True, alpha=0.3, axis='y')

# 小提琴图：Modality分数分布
ax2 = axes[0, 1]
modality_data_list = [df['Ref-Doc'], df['Ref'], df['Doc']]
parts = ax2.violinplot(modality_data_list, positions=[1, 2, 3], showmeans=True, showmedians=True)
ax2.set_xticks([1, 2, 3])
ax2.set_xticklabels(['Ref-Doc', 'Ref', 'Doc'], fontsize=12)
ax2.set_ylabel('Score', fontsize=13, fontweight='bold')
ax2.set_title('Modality分数分布', fontsize=14, fontweight='bold', pad=15)
ax2.grid(True, alpha=0.3, axis='y')

# 柱状图：Top 10模型在三个J-LLM上的表现
ax3 = axes[1, 0]
top10 = df.head(10)
x = np.arange(len(top10))
width = 0.25

bars1 = ax3.bar(x - width, top10['J-LLM-4.1-mini'], width, label='4.1-mini', alpha=0.8, color='#FF6B6B')
bars2 = ax3.bar(x, top10['J-LLM-4.1-nano'], width, label='4.1-nano', alpha=0.8, color='#4ECDC4')
bars3 = ax3.bar(x + width, top10['J-LLM-4o-mini'], width, label='4o-mini', alpha=0.8, color='#95E1D3')

ax3.set_xlabel('Model', fontsize=13, fontweight='bold')
ax3.set_ylabel('Score', fontsize=13, fontweight='bold')
ax3.set_title('Top 10 模型在三个J-LLM上的表现', fontsize=14, fontweight='bold', pad=15)
ax3.set_xticks(x)
ax3.set_xticklabels([m[:15] + '...' if len(m) > 15 else m for m in top10['Model']], 
                     rotation=45, ha='right', fontsize=9)
ax3.legend(fontsize=11)
ax3.grid(True, alpha=0.3, axis='y')

# 折线图：不同模型的J-LLM评估器一致性
ax4 = axes[1, 1]
for idx, row in df.head(15).iterrows():
    jllm_scores = [row['J-LLM-4.1-mini'], row['J-LLM-4.1-nano'], row['J-LLM-4o-mini']]
    ax4.plot([1, 2, 3], jllm_scores, marker='o', alpha=0.6, linewidth=1.5, markersize=6)

ax4.set_xticks([1, 2, 3])
ax4.set_xticklabels(['4.1-mini', '4.1-nano', '4o-mini'], fontsize=12)
ax4.set_ylabel('Score', fontsize=13, fontweight='bold')
ax4.set_title('Top 15 模型的J-LLM评估一致性', fontsize=14, fontweight='bold', pad=15)
ax4.grid(True, alpha=0.3)

plt.tight_layout(pad=3.0)

output_png2 = 'scoring/out/jllm_distribution_analysis.png'
plt.savefig(output_png2, dpi=300, bbox_inches='tight', facecolor='white')
print(f"分布分析图已保存到: {output_png2}")

output_pdf2 = 'scoring/out/jllm_distribution_analysis.pdf'
plt.savefig(output_pdf2, format='pdf', bbox_inches='tight', facecolor='white')
print(f"PDF版本已保存到: {output_pdf2}")

plt.close()

print("\n所有图表生成完成！")
