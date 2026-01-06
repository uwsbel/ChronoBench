"""
Multi-turn Delta Analysis: 分析Turn-to-Turn的分数变化
重点关注编辑能力在不同turn之间的变化
"""
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from scipy import stats
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
    return 'Other'

def load_and_prepare_data():
    """加载数据并准备分析"""
    print("="*80)
    print("Multi-turn Delta Analysis")
    print("="*80)
    
    df = pd.read_csv('scoring/out/all_metrics_merged_pretrain_only.csv')
    
    # 只关注J-LLM-Ref-Doc (score_reference_document)
    df = df[['model', 'system', 'round', 'score_reference_document']].copy()
    
    # 提取round编号
    df['turn'] = df['round'].str.extract(r'round_(\d+)').astype(int)
    
    # 添加类别
    df['category'] = df['system'].apply(get_category)
    
    print(f"\n数据概览:")
    print(f"  模型数: {df['model'].nunique()}")
    print(f"  系统数: {df['system'].nunique()}")
    print(f"  总数据点: {len(df)}")
    
    return df

def calculate_deltas(df):
    """计算Turn-to-Turn Delta"""
    print("\n计算Delta值...")
    
    # Pivot数据：每个(model, system)一行，turn为列
    pivot = df.pivot_table(
        index=['model', 'system', 'category'],
        columns='turn',
        values='score_reference_document'
    ).reset_index()
    
    pivot.columns.name = None
    pivot.columns = ['model', 'system', 'category', 'turn1', 'turn2', 'turn3']
    
    # 计算Delta
    pivot['Δ12'] = pivot['turn2'] - pivot['turn1']  # Turn 1 -> 2
    pivot['Δ23'] = pivot['turn3'] - pivot['turn2']  # Turn 2 -> 3
    pivot['Δ13'] = pivot['turn3'] - pivot['turn1']  # Turn 1 -> 3 (total)
    
    # 移除缺失值
    pivot = pivot.dropna()
    
    print(f"  有效数据点: {len(pivot)}")
    
    return pivot

def compute_statistics(delta_df):
    """计算统计量"""
    print("\n" + "="*80)
    print("全局统计")
    print("="*80)
    
    stats_summary = []
    
    for delta_col in ['Δ12', 'Δ23', 'Δ13']:
        values = delta_df[delta_col]
        
        mean = values.mean()
        std = values.std()
        median = values.median()
        q25 = values.quantile(0.25)
        q75 = values.quantile(0.75)
        
        # 统计显著性检验（t-test: 是否显著不为0）
        t_stat, p_value = stats.ttest_1samp(values, 0)
        
        # 正向/负向/不变的比例
        positive = (values > 0).sum()
        negative = (values < 0).sum()
        unchanged = (values == 0).sum()
        total = len(values)
        
        print(f"\n{delta_col} (Turn变化):")
        print(f"  均值 ± 标准差: {mean:.2f} ± {std:.2f}")
        print(f"  中位数: {median:.2f}")
        print(f"  四分位距: [{q25:.2f}, {q75:.2f}]")
        print(f"  显著性: t={t_stat:.3f}, p={p_value:.6f} {'***' if p_value < 0.001 else '**' if p_value < 0.01 else '*' if p_value < 0.05 else 'ns'}")
        print(f"  方向分布: ↑{positive}({positive/total*100:.1f}%) | ↓{negative}({negative/total*100:.1f}%) | ={unchanged}({unchanged/total*100:.1f}%)")
        
        stats_summary.append({
            'Delta': delta_col,
            'Mean': mean,
            'Std': std,
            'Median': median,
            'Q25': q25,
            'Q75': q75,
            'Positive%': positive/total*100,
            'Negative%': negative/total*100,
            't-stat': t_stat,
            'p-value': p_value
        })
    
    return pd.DataFrame(stats_summary)

def analyze_by_category(delta_df):
    """按系统类别分析"""
    print("\n" + "="*80)
    print("按系统类别分析")
    print("="*80)
    
    category_stats = []
    
    for category in ['MBS', 'FEA', 'VEH', 'SEN', 'RBT']:
        cat_data = delta_df[delta_df['category'] == category]
        
        if len(cat_data) == 0:
            continue
        
        print(f"\n【{category}】(n={len(cat_data)})")
        
        for delta_col in ['Δ12', 'Δ23']:
            values = cat_data[delta_col]
            mean = values.mean()
            std = values.std()
            
            print(f"  {delta_col}: {mean:+.2f} ± {std:.2f}")
            
            category_stats.append({
                'Category': category,
                'Delta': delta_col,
                'Mean': mean,
                'Std': std,
                'N': len(cat_data)
            })
    
    return pd.DataFrame(category_stats)

def analyze_by_model_family(delta_df):
    """按模型家族分析"""
    print("\n" + "="*80)
    print("按模型家族分析")
    print("="*80)
    
    # 提取模型家族
    def get_family(model_name):
        model_lower = model_name.lower()
        if 'llama' in model_lower:
            return 'Llama'
        elif 'gpt' in model_lower:
            return 'GPT'
        elif 'claude' in model_lower:
            return 'Claude'
        elif 'gemini' in model_lower:
            return 'Gemini'
        elif 'deepseek' in model_lower:
            return 'DeepSeek'
        elif 'qwen' in model_lower:
            return 'Qwen'
        elif 'mistral' in model_lower or 'mixtral' in model_lower:
            return 'Mistral'
        elif 'gemma' in model_lower:
            return 'Gemma'
        elif 'phi' in model_lower:
            return 'Phi'
        else:
            return 'Other'
    
    delta_df['family'] = delta_df['model'].apply(get_family)
    
    family_stats = []
    
    for family in sorted(delta_df['family'].unique()):
        fam_data = delta_df[delta_df['family'] == family]
        
        if len(fam_data) < 5:  # 至少5个数据点
            continue
        
        print(f"\n【{family}】(n={len(fam_data)})")
        
        for delta_col in ['Δ12', 'Δ23']:
            values = fam_data[delta_col]
            mean = values.mean()
            std = values.std()
            
            print(f"  {delta_col}: {mean:+.2f} ± {std:.2f}")
            
            family_stats.append({
                'Family': family,
                'Delta': delta_col,
                'Mean': mean,
                'Std': std,
                'N': len(fam_data)
            })
    
    return pd.DataFrame(family_stats), delta_df

def plot_delta_analysis(delta_df, category_stats, family_stats):
    """生成可视化图表"""
    print("\n生成可视化图表...")
    
    # 设置样式
    plt.rcParams['font.size'] = 11
    sns.set_palette("husl")
    
    fig = plt.figure(figsize=(18, 12))
    
    # 1. 全局箱线图
    ax1 = plt.subplot(2, 3, 1)
    data_to_plot = [delta_df['Δ12'], delta_df['Δ23'], delta_df['Δ13']]
    bp = ax1.boxplot(data_to_plot, tick_labels=['Δ12\n(1→2)', 'Δ23\n(2→3)', 'Δ13\n(1→3)'],
                     patch_artist=True, showmeans=True,
                     meanprops=dict(marker='D', markerfacecolor='red', markersize=8))
    
    for patch in bp['boxes']:
        patch.set_facecolor('lightblue')
    
    ax1.axhline(y=0, color='black', linestyle='--', alpha=0.5, linewidth=1.5)
    ax1.set_ylabel('Score Change', fontsize=12, fontweight='bold')
    ax1.set_title('Overall Turn-to-Turn Delta', fontsize=13, fontweight='bold')
    ax1.grid(True, alpha=0.3, axis='y')
    
    # 添加均值标注
    for i, data in enumerate(data_to_plot, 1):
        mean_val = data.mean()
        ax1.text(i, mean_val + 2, f'{mean_val:+.1f}', ha='center', 
                fontsize=10, fontweight='bold', color='red')
    
    # 2. 小提琴图（按类别）
    ax2 = plt.subplot(2, 3, 2)
    
    # 准备数据
    cat_data_list = []
    for cat in ['MBS', 'FEA', 'VEH', 'SEN', 'RBT']:
        cat_df = delta_df[delta_df['category'] == cat]
        for _, row in cat_df.iterrows():
            cat_data_list.append({'Category': cat, 'Delta': 'Δ12', 'Value': row['Δ12']})
            cat_data_list.append({'Category': cat, 'Delta': 'Δ23', 'Value': row['Δ23']})
    
    cat_plot_df = pd.DataFrame(cat_data_list)
    
    sns.violinplot(data=cat_plot_df, x='Category', y='Value', hue='Delta', 
                   split=True, ax=ax2, inner='quartile')
    ax2.axhline(y=0, color='black', linestyle='--', alpha=0.5, linewidth=1.5)
    ax2.set_ylabel('Score Change', fontsize=12, fontweight='bold')
    ax2.set_xlabel('System Category', fontsize=12, fontweight='bold')
    ax2.set_title('Delta by System Category', fontsize=13, fontweight='bold')
    ax2.legend(title='', fontsize=10)
    ax2.grid(True, alpha=0.3, axis='y')
    
    # 3. 按类别的均值对比（柱状图）
    ax3 = plt.subplot(2, 3, 3)
    
    pivot_cat = category_stats.pivot(index='Category', columns='Delta', values='Mean')
    pivot_cat.plot(kind='bar', ax=ax3, color=['#FF6B6B', '#4ECDC4'], alpha=0.8, width=0.7)
    
    ax3.axhline(y=0, color='black', linestyle='-', alpha=0.3, linewidth=1)
    ax3.set_ylabel('Mean Score Change', fontsize=12, fontweight='bold')
    ax3.set_xlabel('System Category', fontsize=12, fontweight='bold')
    ax3.set_title('Mean Delta by Category', fontsize=13, fontweight='bold')
    ax3.legend(title='', fontsize=10)
    ax3.set_xticklabels(ax3.get_xticklabels(), rotation=45, ha='right')
    ax3.grid(True, alpha=0.3, axis='y')
    
    # 4. 散点图：Δ12 vs Δ23
    ax4 = plt.subplot(2, 3, 4)
    
    colors = {'MBS': '#FF6B6B', 'FEA': '#4ECDC4', 'VEH': '#95E1D3', 
              'SEN': '#FFD93D', 'RBT': '#A8E6CF'}
    
    for cat in ['MBS', 'FEA', 'VEH', 'SEN', 'RBT']:
        cat_df = delta_df[delta_df['category'] == cat]
        ax4.scatter(cat_df['Δ12'], cat_df['Δ23'], 
                   alpha=0.6, s=60, c=colors[cat], label=cat, edgecolors='black', linewidths=0.5)
    
    # 添加象限线
    ax4.axhline(y=0, color='black', linestyle='-', alpha=0.3, linewidth=1)
    ax4.axvline(x=0, color='black', linestyle='-', alpha=0.3, linewidth=1)
    
    # 添加对角线
    lims = [
        np.min([ax4.get_xlim(), ax4.get_ylim()]),
        np.max([ax4.get_xlim(), ax4.get_ylim()]),
    ]
    ax4.plot(lims, lims, 'k--', alpha=0.3, zorder=0, linewidth=1)
    
    ax4.set_xlabel('Δ12 (Turn 1→2 Improvement)', fontsize=12, fontweight='bold')
    ax4.set_ylabel('Δ23 (Turn 2→3 Change)', fontsize=12, fontweight='bold')
    ax4.set_title('Delta Correlation', fontsize=13, fontweight='bold')
    ax4.legend(fontsize=9, loc='best')
    ax4.grid(True, alpha=0.3)
    
    # 5. 按模型家族的箱线图
    ax5 = plt.subplot(2, 3, 5)
    
    # 只显示主要家族
    main_families = ['Llama', 'GPT', 'Claude', 'Gemini', 'Mistral', 'DeepSeek']
    fam_data_list = []
    for fam in main_families:
        fam_df = delta_df[delta_df['family'] == fam]
        if len(fam_df) >= 5:
            for _, row in fam_df.iterrows():
                fam_data_list.append({'Family': fam, 'Delta': 'Δ12', 'Value': row['Δ12']})
                fam_data_list.append({'Family': fam, 'Delta': 'Δ23', 'Value': row['Δ23']})
    
    if len(fam_data_list) > 0:
        fam_plot_df = pd.DataFrame(fam_data_list)
        sns.boxplot(data=fam_plot_df, x='Family', y='Value', hue='Delta', ax=ax5)
        ax5.axhline(y=0, color='black', linestyle='--', alpha=0.5, linewidth=1.5)
        ax5.set_ylabel('Score Change', fontsize=12, fontweight='bold')
        ax5.set_xlabel('Model Family', fontsize=12, fontweight='bold')
        ax5.set_title('Delta by Model Family', fontsize=13, fontweight='bold')
        ax5.legend(title='', fontsize=10)
        ax5.set_xticklabels(ax5.get_xticklabels(), rotation=45, ha='right')
        ax5.grid(True, alpha=0.3, axis='y')
    
    # 6. 累积分布函数
    ax6 = plt.subplot(2, 3, 6)
    
    for delta_col, color, label in [('Δ12', '#FF6B6B', 'Turn 1→2'), 
                                     ('Δ23', '#4ECDC4', 'Turn 2→3')]:
        values = np.sort(delta_df[delta_col])
        cdf = np.arange(1, len(values)+1) / len(values)
        ax6.plot(values, cdf, linewidth=2.5, alpha=0.8, color=color, label=label)
    
    ax6.axvline(x=0, color='black', linestyle='--', alpha=0.5, linewidth=1.5)
    ax6.set_xlabel('Score Change', fontsize=12, fontweight='bold')
    ax6.set_ylabel('Cumulative Probability', fontsize=12, fontweight='bold')
    ax6.set_title('Cumulative Distribution', fontsize=13, fontweight='bold')
    ax6.legend(fontsize=11)
    ax6.grid(True, alpha=0.3)
    
    plt.suptitle('Multi-turn Delta Analysis: Turn-to-Turn Score Changes', 
                 fontsize=16, fontweight='bold', y=0.995)
    
    plt.tight_layout(rect=[0, 0, 1, 0.99])
    
    output_png = 'scoring/out/multiturn_delta_analysis.png'
    plt.savefig(output_png, dpi=300, bbox_inches='tight', facecolor='white')
    print(f"图表已保存到: {output_png}")
    
    output_pdf = 'scoring/out/multiturn_delta_analysis.pdf'
    plt.savefig(output_pdf, format='pdf', bbox_inches='tight', facecolor='white')
    print(f"PDF已保存到: {output_pdf}")
    
    plt.close()

def save_results(delta_df, global_stats, category_stats, family_stats):
    """保存分析结果"""
    print("\n保存分析结果...")
    
    # 1. Delta数据
    delta_df.to_csv('scoring/out/multiturn_delta_data.csv', index=False, float_format='%.2f')
    print("  Delta数据已保存: multiturn_delta_data.csv")
    
    # 2. 全局统计
    global_stats.to_csv('scoring/out/multiturn_delta_global_stats.csv', index=False, float_format='%.4f')
    print("  全局统计已保存: multiturn_delta_global_stats.csv")
    
    # 3. 类别统计
    category_stats.to_csv('scoring/out/multiturn_delta_category_stats.csv', index=False, float_format='%.2f')
    print("  类别统计已保存: multiturn_delta_category_stats.csv")
    
    # 4. 家族统计
    family_stats.to_csv('scoring/out/multiturn_delta_family_stats.csv', index=False, float_format='%.2f')
    print("  家族统计已保存: multiturn_delta_family_stats.csv")

def main():
    # 加载数据
    df = load_and_prepare_data()
    
    # 计算Delta
    delta_df = calculate_deltas(df)
    
    # 全局统计
    global_stats = compute_statistics(delta_df)
    
    # 按类别分析
    category_stats = analyze_by_category(delta_df)
    
    # 按模型家族分析
    family_stats, delta_df = analyze_by_model_family(delta_df)
    
    # 可视化
    plot_delta_analysis(delta_df, category_stats, family_stats)
    
    # 保存结果
    save_results(delta_df, global_stats, category_stats, family_stats)
    
    print("\n" + "="*80)
    print("分析完成！")
    print("="*80)

if __name__ == "__main__":
    main()
