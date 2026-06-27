"""
Failure Mode Analysis: 分析34个系统的难度和失败模式
找出最难的系统，分析不同类别的挑战
"""
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import sys
import io

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')

# 系统分类（来自evaluatePy.py的正确分类）
SYSTEM_CATEGORIES = {
    'MBS': ['pendulum', 'slider_crank', 'gear', 'mass_spring_damper', 'particles'],
    'FEA': ['beam', 'buckling', 'rotor', 'tablecloth', 'cable'],
    'SEN': ['gps_imu', 'lidar', 'veh_app', 'camera'],
    'RBT': ['turtlebot', 'viper', 'curiosity', 'vehros', 'sensros', 'handler'],
    'VEH': ['citybus', 'feda', 'gator', 'hmmwv', 'kraz', 'art', 'rigid_highway', 
            'rigid_multipatches', 'scm', 'scm_hill', 'uazbus', 'm113', 'sedan', 'man']
}

def get_category(system_name):
    """获取系统类别"""
    for cat, systems in SYSTEM_CATEGORIES.items():
        if system_name in systems:
            return cat
    return 'UNK'

def load_and_analyze():
    """加载数据并分析"""
    print("="*80)
    print("Failure Mode Analysis: System Difficulty Assessment")
    print("="*80)
    
    df = pd.read_csv('paper/out/all_metrics_merged_pretrain_only.csv')
    
    # 只关注J-LLM-Ref-Doc
    df = df[['model', 'system', 'round', 'score_reference_document']].copy()
    
    # 提取turn编号
    df['turn'] = df['round'].str.extract(r'round_(\d+)').astype(int)
    
    print(f"\n数据概览:")
    print(f"  模型数: {df['model'].nunique()}")
    print(f"  系统数: {df['system'].nunique()}")
    print(f"  总数据点: {len(df)}")
    
    return df

def compute_system_statistics(df):
    """计算每个系统的统计数据"""
    print("\n计算系统级统计...")
    
    # 1. 每个系统在每个turn上的平均分（跨所有模型）
    turn_avg = df.groupby(['system', 'turn'])['score_reference_document'].mean().reset_index()
    turn_avg_pivot = turn_avg.pivot(index='system', columns='turn', values='score_reference_document')
    turn_avg_pivot.columns = ['Turn1_Avg', 'Turn2_Avg', 'Turn3_Avg']
    
    # 2. 每个系统的总平均分（跨所有模型和所有turn）
    system_overall = df.groupby('system')['score_reference_document'].agg([
        ('Overall_Avg', 'mean'),
        ('Overall_Std', 'std'),
        ('Overall_Min', 'min'),
        ('Overall_Max', 'max')
    ]).reset_index()
    
    # 3. 合并
    system_stats = turn_avg_pivot.reset_index().merge(system_overall, on='system')
    
    # 4. 添加类别
    system_stats['Category'] = system_stats['system'].apply(get_category)
    
    # 5. 计算turn-to-turn变化
    system_stats['Δ12'] = system_stats['Turn2_Avg'] - system_stats['Turn1_Avg']
    system_stats['Δ23'] = system_stats['Turn3_Avg'] - system_stats['Turn2_Avg']
    
    # 6. 计算难度指标（分数越低越难）
    system_stats['Difficulty_Score'] = system_stats['Overall_Avg']
    
    # 7. 排序（从最难到最容易）
    system_stats = system_stats.sort_values('Difficulty_Score')
    
    # 8. 添加排名
    system_stats['Rank'] = range(1, len(system_stats) + 1)
    
    # 重新排列列顺序
    system_stats = system_stats[[
        'Rank', 'system', 'Category', 
        'Turn1_Avg', 'Turn2_Avg', 'Turn3_Avg', 
        'Overall_Avg', 'Overall_Std',
        'Δ12', 'Δ23',
        'Overall_Min', 'Overall_Max'
    ]]
    
    return system_stats

def analyze_failure_modes(system_stats):
    """分析失败模式"""
    print("\n" + "="*80)
    print("失败模式分析")
    print("="*80)
    
    # 1. 最难的10个系统
    print("\n【Top 10 最难系统】(Overall_Avg最低)")
    hardest_10 = system_stats.head(10)
    for _, row in hardest_10.iterrows():
        print(f"  {row['Rank']:2d}. {row['system']:25s} ({row['Category']:3s}): "
              f"Avg={row['Overall_Avg']:5.1f}, T1={row['Turn1_Avg']:4.1f}, "
              f"T2={row['Turn2_Avg']:4.1f}, T3={row['Turn3_Avg']:4.1f}")
    
    # 2. 最容易的10个系统
    print("\n【Top 10 最容易系统】(Overall_Avg最高)")
    easiest_10 = system_stats.tail(10)
    for _, row in easiest_10.iterrows():
        print(f"  {row['Rank']:2d}. {row['system']:25s} ({row['Category']:3s}): "
              f"Avg={row['Overall_Avg']:5.1f}, T1={row['Turn1_Avg']:4.1f}, "
              f"T2={row['Turn2_Avg']:4.1f}, T3={row['Turn3_Avg']:4.1f}")
    
    # 3. 按类别分析
    print("\n【按类别平均难度】")
    category_stats = system_stats.groupby('Category').agg({
        'Overall_Avg': 'mean',
        'Turn1_Avg': 'mean',
        'Turn2_Avg': 'mean',
        'Turn3_Avg': 'mean',
        'Δ12': 'mean',
        'Δ23': 'mean'
    }).sort_values('Overall_Avg')
    
    for cat, row in category_stats.iterrows():
        n_systems = len(system_stats[system_stats['Category'] == cat])
        print(f"  {cat:3s} (n={n_systems:2d}): Avg={row['Overall_Avg']:5.1f}, "
              f"T1={row['Turn1_Avg']:4.1f}, T2={row['Turn2_Avg']:4.1f}, "
              f"T3={row['Turn3_Avg']:4.1f}")
    
    # 4. 特殊失败模式
    print("\n【特殊失败模式】")
    
    # Turn 1表现最差
    worst_turn1 = system_stats.nsmallest(5, 'Turn1_Avg')
    print("\n  Turn 1 最难（从零生成）:")
    for _, row in worst_turn1.iterrows():
        print(f"    {row['system']:25s}: {row['Turn1_Avg']:4.1f}")
    
    # Turn 2提升最少
    smallest_delta12 = system_stats.nsmallest(5, 'Δ12')
    print("\n  Turn 2 提升最少（上下文帮助最小）:")
    for _, row in smallest_delta12.iterrows():
        print(f"    {row['system']:25s}: Δ12={row['Δ12']:+5.1f}")
    
    # Turn 3下降最严重
    worst_delta23 = system_stats.nsmallest(5, 'Δ23')
    print("\n  Turn 3 下降最严重（扩展最难）:")
    for _, row in worst_delta23.iterrows():
        print(f"    {row['system']:25s}: Δ23={row['Δ23']:+5.1f}")
    
    # 方差最大（最不稳定）
    most_unstable = system_stats.nlargest(5, 'Overall_Std')
    print("\n  最不稳定（模型间差异最大）:")
    for _, row in most_unstable.iterrows():
        print(f"    {row['system']:25s}: Std={row['Overall_Std']:4.1f}")
    
    return category_stats

def plot_failure_analysis(system_stats, category_stats):
    """可视化失败模式分析"""
    print("\n生成可视化...")
    
    plt.rcParams['font.size'] = 10
    fig = plt.figure(figsize=(18, 14))
    
    # 1. 系统难度排名（前20）
    ax1 = plt.subplot(3, 2, 1)
    top20 = system_stats.head(20)
    
    y_pos = np.arange(len(top20))
    colors = [{'MBS': '#FF6B6B', 'FEA': '#4ECDC4', 'VEH': '#95E1D3', 
               'SEN': '#FFD93D', 'RBT': '#A8E6CF', 'SCM': '#C7CEEA',
               'RIG': '#FFDAC1', 'OTH': '#B5EAD7'}.get(cat, '#CCCCCC') 
              for cat in top20['Category']]
    
    bars = ax1.barh(y_pos, top20['Overall_Avg'], color=colors, alpha=0.8, edgecolor='black', linewidth=0.5)
    ax1.set_yticks(y_pos)
    ax1.set_yticklabels(top20['system'], fontsize=9)
    ax1.set_xlabel('Average Score (Lower = Harder)', fontsize=11, fontweight='bold')
    ax1.set_title('Top 20 Most Difficult Systems', fontsize=12, fontweight='bold')
    ax1.invert_yaxis()
    ax1.grid(True, alpha=0.3, axis='x')
    
    # 添加分数标签
    for i, (bar, score) in enumerate(zip(bars, top20['Overall_Avg'])):
        ax1.text(score + 0.5, i, f'{score:.1f}', va='center', fontsize=8)
    
    # 2. 按类别的平均难度
    ax2 = plt.subplot(3, 2, 2)
    
    cat_sorted = category_stats.sort_values('Overall_Avg')
    colors_cat = ['#FF6B6B', '#4ECDC4', '#95E1D3', '#FFD93D', '#A8E6CF', '#C7CEEA', '#FFDAC1', '#B5EAD7']
    
    bars = ax2.bar(cat_sorted.index, cat_sorted['Overall_Avg'], 
                   color=colors_cat[:len(cat_sorted)], alpha=0.8, edgecolor='black', linewidth=1)
    ax2.set_ylabel('Average Score', fontsize=11, fontweight='bold')
    ax2.set_xlabel('Category', fontsize=11, fontweight='bold')
    ax2.set_title('Difficulty by System Category', fontsize=12, fontweight='bold')
    ax2.grid(True, alpha=0.3, axis='y')
    
    for bar, score in zip(bars, cat_sorted['Overall_Avg']):
        ax2.text(bar.get_x() + bar.get_width()/2, score + 1, f'{score:.1f}',
                ha='center', fontsize=10, fontweight='bold')
    
    # 3. Turn-by-turn表现（最难的10个系统）
    ax3 = plt.subplot(3, 2, 3)
    
    hardest_10 = system_stats.head(10)
    x = np.arange(len(hardest_10))
    width = 0.25
    
    bars1 = ax3.bar(x - width, hardest_10['Turn1_Avg'], width, label='Turn 1', alpha=0.8, color='#FF6B6B')
    bars2 = ax3.bar(x, hardest_10['Turn2_Avg'], width, label='Turn 2', alpha=0.8, color='#4ECDC4')
    bars3 = ax3.bar(x + width, hardest_10['Turn3_Avg'], width, label='Turn 3', alpha=0.8, color='#95E1D3')
    
    ax3.set_ylabel('Average Score', fontsize=11, fontweight='bold')
    ax3.set_xlabel('System', fontsize=11, fontweight='bold')
    ax3.set_title('Turn-by-Turn Performance: 10 Hardest Systems', fontsize=12, fontweight='bold')
    ax3.set_xticks(x)
    ax3.set_xticklabels([s[:15] for s in hardest_10['system']], rotation=45, ha='right', fontsize=8)
    ax3.legend(fontsize=10)
    ax3.grid(True, alpha=0.3, axis='y')
    
    # 4. Delta分布（Δ12 vs Δ23）
    ax4 = plt.subplot(3, 2, 4)
    
    categories = system_stats['Category'].unique()
    cat_colors = {'MBS': '#FF6B6B', 'FEA': '#4ECDC4', 'VEH': '#95E1D3', 
                  'SEN': '#FFD93D', 'RBT': '#A8E6CF', 'SCM': '#C7CEEA',
                  'RIG': '#FFDAC1', 'OTH': '#B5EAD7'}
    
    for cat in categories:
        cat_data = system_stats[system_stats['Category'] == cat]
        ax4.scatter(cat_data['Δ12'], cat_data['Δ23'], 
                   s=100, alpha=0.7, c=cat_colors.get(cat, '#CCCCCC'), 
                   label=cat, edgecolors='black', linewidths=0.5)
    
    ax4.axhline(y=0, color='black', linestyle='--', alpha=0.5, linewidth=1)
    ax4.axvline(x=0, color='black', linestyle='--', alpha=0.5, linewidth=1)
    ax4.set_xlabel('Δ12 (Turn 1→2 Improvement)', fontsize=11, fontweight='bold')
    ax4.set_ylabel('Δ23 (Turn 2→3 Change)', fontsize=11, fontweight='bold')
    ax4.set_title('System Delta Patterns', fontsize=12, fontweight='bold')
    ax4.legend(fontsize=9, ncol=2)
    ax4.grid(True, alpha=0.3)
    
    # 5. 标准差分析（模型间差异）
    ax5 = plt.subplot(3, 2, 5)
    
    top15_unstable = system_stats.nlargest(15, 'Overall_Std').sort_values('Overall_Std', ascending=False)
    
    y_pos = np.arange(len(top15_unstable))
    bars = ax5.barh(y_pos, top15_unstable['Overall_Std'], 
                    color='#FF6B6B', alpha=0.7, edgecolor='black', linewidth=0.5)
    ax5.set_yticks(y_pos)
    ax5.set_yticklabels(top15_unstable['system'], fontsize=9)
    ax5.set_xlabel('Standard Deviation', fontsize=11, fontweight='bold')
    ax5.set_title('Most Inconsistent Systems (High Model Variance)', fontsize=12, fontweight='bold')
    ax5.invert_yaxis()
    ax5.grid(True, alpha=0.3, axis='x')
    
    # 6. 类别的turn-by-turn模式
    ax6 = plt.subplot(3, 2, 6)
    
    cat_sorted = category_stats.sort_values('Overall_Avg')
    x = np.arange(len(cat_sorted))
    width = 0.25
    
    bars1 = ax6.bar(x - width, cat_sorted['Turn1_Avg'], width, label='Turn 1', alpha=0.8, color='#FF6B6B')
    bars2 = ax6.bar(x, cat_sorted['Turn2_Avg'], width, label='Turn 2', alpha=0.8, color='#4ECDC4')
    bars3 = ax6.bar(x + width, cat_sorted['Turn3_Avg'], width, label='Turn 3', alpha=0.8, color='#95E1D3')
    
    ax6.set_ylabel('Average Score', fontsize=11, fontweight='bold')
    ax6.set_xlabel('Category', fontsize=11, fontweight='bold')
    ax6.set_title('Category Turn-by-Turn Performance', fontsize=12, fontweight='bold')
    ax6.set_xticks(x)
    ax6.set_xticklabels(cat_sorted.index, fontsize=10)
    ax6.legend(fontsize=10)
    ax6.grid(True, alpha=0.3, axis='y')
    
    plt.suptitle('Failure Mode Analysis: System Difficulty Assessment', 
                 fontsize=15, fontweight='bold', y=0.995)
    
    plt.tight_layout(rect=[0, 0, 1, 0.99])
    
    output_png = 'paper/out/failure_mode_analysis.png'
    plt.savefig(output_png, dpi=300, bbox_inches='tight', facecolor='white')
    print(f"图表已保存到: {output_png}")
    
    output_pdf = 'paper/out/failure_mode_analysis.pdf'
    plt.savefig(output_pdf, format='pdf', bbox_inches='tight', facecolor='white')
    print(f"PDF已保存到: {output_pdf}")
    
    plt.close()

def save_results(system_stats, category_stats):
    """保存结果"""
    print("\n保存分析结果...")
    
    # 1. 系统统计
    system_stats.to_csv('paper/out/system_difficulty_analysis.csv', 
                        index=False, float_format='%.2f')
    print("  系统难度分析已保存: system_difficulty_analysis.csv")
    
    # 2. 类别统计
    category_stats.to_csv('paper/out/category_difficulty_analysis.csv', 
                         float_format='%.2f')
    print("  类别难度分析已保存: category_difficulty_analysis.csv")

def main():
    # 加载数据
    df = load_and_analyze()
    
    # 计算系统统计
    system_stats = compute_system_statistics(df)
    
    # 分析失败模式
    category_stats = analyze_failure_modes(system_stats)
    
    # 可视化
    plot_failure_analysis(system_stats, category_stats)
    
    # 保存结果
    save_results(system_stats, category_stats)
    
    print("\n" + "="*80)
    print("失败模式分析完成！")
    print("="*80)
    
    return system_stats, category_stats

if __name__ == "__main__":
    system_stats, category_stats = main()
