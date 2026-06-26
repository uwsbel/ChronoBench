"""
SimBench LLM Benchmark Data Analysis
===================================
Analyze combined_evaluation_scores.csv for paper statistics and visualizations

Author: SimBench Team
Date: 2026-01-02
"""

import sys
import io
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from scipy import stats
from pathlib import Path
import warnings
warnings.filterwarnings('ignore')

# 设置中文字体和样式
plt.rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans', 'Arial Unicode MS']
plt.rcParams['axes.unicode_minus'] = False
plt.rcParams['figure.dpi'] = 150
plt.rcParams['savefig.dpi'] = 300

# 设置seaborn样式
sns.set_style("whitegrid")
sns.set_palette("husl")

# 输出目录
OUTPUT_DIR = Path(__file__).parent / "analysis_output"
OUTPUT_DIR.mkdir(exist_ok=True)

def load_data():
    """加载并预处理数据"""
    data_path = Path(__file__).parent.parent / "output_llms" / "combined_evaluation_scores.csv"
    df = pd.read_csv(data_path)
    
    # 重命名列
    df.columns = ['Model', 'System', 'Round', 'Score_Doc', 'Score_Ref', 'Score_RefDoc']
    
    # 添加综合得分
    df['Score_Avg'] = (df['Score_Doc'] + df['Score_Ref'] + df['Score_RefDoc']) / 3
    
    # 将Round转换为数值便于分析
    round_map = {'first': 1, 'second': 2, 'third': 3}
    df['Round_Num'] = df['Round'].map(round_map)
    
    print(f"✅ 数据加载完成!")
    print(f"   - 总记录数: {len(df)}")
    print(f"   - 模型数量: {df['Model'].nunique()}")
    print(f"   - 仿真系统数量: {df['System'].nunique()}")
    print(f"   - 测试轮次: {df['Round'].unique().tolist()}")
    
    return df

def analyze_model_ranking(df):
    """
    分析1: 模型整体性能排名
    """
    print("\n" + "="*60)
    print("📊 分析1: 模型整体性能排名")
    print("="*60)
    
    # 计算每个模型的统计指标
    model_stats = df.groupby('Model').agg({
        'Score_Doc': ['mean', 'std', 'median'],
        'Score_Ref': ['mean', 'std', 'median'],
        'Score_RefDoc': ['mean', 'std', 'median'],
        'Score_Avg': ['mean', 'std', 'median', 'min', 'max']
    }).round(2)
    
    # 扁平化列名
    model_stats.columns = ['_'.join(col).strip() for col in model_stats.columns.values]
    model_stats = model_stats.sort_values('Score_Avg_mean', ascending=False)
    
    # 添加排名
    model_stats['Rank'] = range(1, len(model_stats) + 1)
    
    # 保存到CSV
    model_stats.to_csv(OUTPUT_DIR / "01_model_ranking.csv")
    
    # 打印Top 10
    print("\n🏆 Top 10 模型排名 (按平均综合得分):")
    print("-" * 80)
    top10 = model_stats.head(10)[['Rank', 'Score_Avg_mean', 'Score_Avg_std', 'Score_Doc_mean', 'Score_Ref_mean', 'Score_RefDoc_mean']]
    print(top10.to_string())
    
    # 创建排名可视化
    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    
    # 1. 条形图 - 模型平均得分
    ax1 = axes[0, 0]
    top_models = model_stats.head(15).index.tolist()
    scores = model_stats.loc[top_models, 'Score_Avg_mean']
    stds = model_stats.loc[top_models, 'Score_Avg_std']
    colors = plt.cm.RdYlGn(np.linspace(0.8, 0.2, len(top_models)))
    bars = ax1.barh(range(len(top_models)), scores, xerr=stds, color=colors, capsize=3)
    ax1.set_yticks(range(len(top_models)))
    ax1.set_yticklabels(top_models)
    ax1.invert_yaxis()
    ax1.set_xlabel('Average Score')
    ax1.set_title('Top 15 Models by Average Score', fontsize=12, fontweight='bold')
    ax1.axvline(x=scores.mean(), color='red', linestyle='--', alpha=0.7, label=f'Mean: {scores.mean():.1f}')
    ax1.legend()
    
    # 2. 箱线图 - 所有模型得分分布
    ax2 = axes[0, 1]
    model_order = model_stats.head(15).index.tolist()
    df_top = df[df['Model'].isin(model_order)]
    sns.boxplot(data=df_top, x='Score_Avg', y='Model', order=model_order, ax=ax2, palette='RdYlGn_r')
    ax2.set_xlabel('Average Score')
    ax2.set_title('Score Distribution of Top 15 Models', fontsize=12, fontweight='bold')
    
    # 3. 三个评分维度对比
    ax3 = axes[1, 0]
    x = np.arange(len(top_models[:10]))
    width = 0.25
    ax3.bar(x - width, model_stats.loc[top_models[:10], 'Score_Doc_mean'], width, label='Score Document', color='#3498db')
    ax3.bar(x, model_stats.loc[top_models[:10], 'Score_Ref_mean'], width, label='Score Reference', color='#2ecc71')
    ax3.bar(x + width, model_stats.loc[top_models[:10], 'Score_RefDoc_mean'], width, label='Score RefDoc', color='#e74c3c')
    ax3.set_xticks(x)
    ax3.set_xticklabels(top_models[:10], rotation=45, ha='right')
    ax3.set_ylabel('Score')
    ax3.set_title('Three Scoring Dimensions Comparison (Top 10)', fontsize=12, fontweight='bold')
    ax3.legend()
    
    # 4. 稳定性分析 - 均值 vs 标准差
    ax4 = axes[1, 1]
    scatter = ax4.scatter(model_stats['Score_Avg_mean'], model_stats['Score_Avg_std'], 
                         c=model_stats['Rank'], cmap='RdYlGn_r', s=100, alpha=0.7)
    plt.colorbar(scatter, ax=ax4, label='Rank')
    ax4.set_xlabel('Average Score (Mean)')
    ax4.set_ylabel('Score Variability (Std)')
    ax4.set_title('Model Performance vs Stability', fontsize=12, fontweight='bold')
    
    # 标注Top 5
    for model in model_stats.head(5).index:
        ax4.annotate(model, (model_stats.loc[model, 'Score_Avg_mean'], 
                            model_stats.loc[model, 'Score_Avg_std']),
                    fontsize=8, alpha=0.8)
    
    plt.tight_layout()
    plt.savefig(OUTPUT_DIR / "01_model_ranking.png", bbox_inches='tight')
    plt.close()
    
    print(f"\n✅ 结果已保存到 {OUTPUT_DIR}/01_model_ranking.csv 和 .png")
    
    return model_stats

def analyze_round_progression(df):
    """
    分析2: 多轮对话效果分析
    """
    print("\n" + "="*60)
    print("📊 分析2: 多轮对话效果分析")
    print("="*60)
    
    # 计算每个模型在每轮的平均得分
    round_stats = df.groupby(['Model', 'Round']).agg({
        'Score_Avg': 'mean',
        'Score_Doc': 'mean',
        'Score_Ref': 'mean',
        'Score_RefDoc': 'mean'
    }).round(2)
    
    # 透视表
    pivot_avg = df.pivot_table(values='Score_Avg', index='Model', columns='Round', aggfunc='mean')
    pivot_avg = pivot_avg[['first', 'second', 'third']]  # 确保顺序
    
    # 计算改进幅度
    pivot_avg['Improvement_1to2'] = pivot_avg['second'] - pivot_avg['first']
    pivot_avg['Improvement_2to3'] = pivot_avg['third'] - pivot_avg['second']
    pivot_avg['Total_Improvement'] = pivot_avg['third'] - pivot_avg['first']
    pivot_avg = pivot_avg.round(2)
    
    # 保存
    pivot_avg.to_csv(OUTPUT_DIR / "02_round_progression.csv")
    
    # 打印改进最大的模型
    print("\n📈 多轮改进最大的 Top 10 模型:")
    print("-" * 60)
    top_improvers = pivot_avg.sort_values('Total_Improvement', ascending=False).head(10)
    print(top_improvers[['first', 'second', 'third', 'Total_Improvement']].to_string())
    
    print("\n📉 多轮退化最明显的 Top 5 模型:")
    print("-" * 60)
    worst_improvers = pivot_avg.sort_values('Total_Improvement', ascending=True).head(5)
    print(worst_improvers[['first', 'second', 'third', 'Total_Improvement']].to_string())
    
    # 可视化
    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    
    # 1. 所有模型的多轮趋势热力图
    ax1 = axes[0, 0]
    # 按总体表现排序
    sorted_models = pivot_avg.sort_values('first', ascending=False).index[:20]
    heatmap_data = pivot_avg.loc[sorted_models, ['first', 'second', 'third']]
    sns.heatmap(heatmap_data, annot=True, fmt='.1f', cmap='RdYlGn', ax=ax1, 
                cbar_kws={'label': 'Average Score'})
    ax1.set_title('Score Progression Across Rounds (Top 20 Models)', fontsize=12, fontweight='bold')
    ax1.set_xlabel('Round')
    ax1.set_ylabel('Model')
    
    # 2. 折线图 - Top 10 模型的多轮趋势
    ax2 = axes[0, 1]
    top10_models = pivot_avg.sort_values('first', ascending=False).head(10).index
    for model in top10_models:
        scores = pivot_avg.loc[model, ['first', 'second', 'third']]
        ax2.plot(['Round 1', 'Round 2', 'Round 3'], scores, marker='o', label=model, linewidth=2)
    ax2.set_ylabel('Average Score')
    ax2.set_title('Top 10 Models Performance Across Rounds', fontsize=12, fontweight='bold')
    ax2.legend(bbox_to_anchor=(1.05, 1), loc='upper left', fontsize=8)
    ax2.grid(True, alpha=0.3)
    
    # 3. 改进幅度分布
    ax3 = axes[1, 0]
    improvements = pivot_avg['Total_Improvement'].dropna()
    colors = ['green' if x > 0 else 'red' for x in improvements]
    ax3.hist(improvements, bins=20, color='steelblue', edgecolor='white', alpha=0.7)
    ax3.axvline(x=0, color='red', linestyle='--', linewidth=2, label='No Change')
    ax3.axvline(x=improvements.mean(), color='green', linestyle='--', linewidth=2, 
               label=f'Mean: {improvements.mean():.1f}')
    ax3.set_xlabel('Total Improvement (Round 3 - Round 1)')
    ax3.set_ylabel('Number of Models')
    ax3.set_title('Distribution of Multi-Round Improvement', fontsize=12, fontweight='bold')
    ax3.legend()
    
    # 4. 每轮平均得分统计
    ax4 = axes[1, 1]
    round_means = df.groupby('Round')['Score_Avg'].agg(['mean', 'std']).loc[['first', 'second', 'third']]
    x = ['Round 1', 'Round 2', 'Round 3']
    bars = ax4.bar(x, round_means['mean'], yerr=round_means['std'], 
                   color=['#3498db', '#2ecc71', '#e74c3c'], capsize=5, alpha=0.8)
    ax4.set_ylabel('Average Score')
    ax4.set_title('Overall Performance by Round', fontsize=12, fontweight='bold')
    
    # 添加数值标签
    for bar, val in zip(bars, round_means['mean']):
        ax4.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 1, 
                f'{val:.1f}', ha='center', fontsize=12, fontweight='bold')
    
    plt.tight_layout()
    plt.savefig(OUTPUT_DIR / "02_round_progression.png", bbox_inches='tight')
    plt.close()
    
    print(f"\n✅ 结果已保存到 {OUTPUT_DIR}/02_round_progression.csv 和 .png")
    
    return pivot_avg

def analyze_task_difficulty(df):
    """
    分析3: 任务/系统难度分析
    """
    print("\n" + "="*60)
    print("📊 分析3: 仿真系统难度分析")
    print("="*60)
    
    # 计算每个系统的统计指标
    system_stats = df.groupby('System').agg({
        'Score_Avg': ['mean', 'std', 'min', 'max', 'count'],
        'Score_Doc': 'mean',
        'Score_Ref': 'mean',
        'Score_RefDoc': 'mean'
    }).round(2)
    
    system_stats.columns = ['_'.join(col).strip() for col in system_stats.columns.values]
    system_stats = system_stats.sort_values('Score_Avg_mean', ascending=False)
    system_stats['Difficulty_Rank'] = range(1, len(system_stats) + 1)
    
    # 保存
    system_stats.to_csv(OUTPUT_DIR / "03_task_difficulty.csv")
    
    # 分类系统 (基于官方分类)
    # MBS: Multi-Body Systems (多体系统)
    MBS_list = ['pendulum', 'slider_crank', 'gear', 'mass_spring_damper', 'particles']
    # FEA: Finite Element Analysis (有限元分析)
    FEA_list = ['beam', 'buckling', 'rotor', 'tablecloth', 'cable']
    # SEN: Sensors (传感器)
    SEN_list = ['gps_imu', 'lidar', 'veh_app', 'camera']
    # RBT: Robotics (机器人)
    RBT_list = ['turtlebot', 'viper', 'curiosity', 'vehros', 'sensros', 'handler']
    # VEH: Vehicles (车辆)
    VEH_list = ['citybus', 'feda', 'gator', 'hmmwv', 'kraz', 'art', 'rigid_highway', 
                'rigid_multipatches', 'scm', 'scm_hill', 'uazbus', 'm113', 'sedan', 'man']
    
    def categorize_system(sys_name):
        if sys_name in MBS_list:
            return 'MBS'
        elif sys_name in FEA_list:
            return 'FEA'
        elif sys_name in SEN_list:
            return 'Sensor'
        elif sys_name in RBT_list:
            return 'Robot'
        elif sys_name in VEH_list:
            return 'Vehicle'
        else:
            return 'Other'
    
    df['Category'] = df['System'].apply(categorize_system)
    
    print("\n🎯 最容易的 Top 5 系统 (平均得分最高):")
    print("-" * 60)
    print(system_stats.head(5)[['Score_Avg_mean', 'Score_Avg_std', 'Difficulty_Rank']].to_string())
    
    print("\n🔥 最难的 Top 5 系统 (平均得分最低):")
    print("-" * 60)
    print(system_stats.tail(5)[['Score_Avg_mean', 'Score_Avg_std', 'Difficulty_Rank']].to_string())
    
    # 按类别统计
    category_stats = df.groupby('Category')['Score_Avg'].agg(['mean', 'std', 'count']).round(2)
    print("\n📊 按类别统计:")
    print("-" * 40)
    print(category_stats.to_string())
    
    # 可视化
    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    
    # 1. 系统难度条形图
    ax1 = axes[0, 0]
    systems = system_stats.index.tolist()
    scores = system_stats['Score_Avg_mean'].values
    colors = plt.cm.RdYlGn(scores / 100)
    bars = ax1.barh(range(len(systems)), scores, color=colors)
    ax1.set_yticks(range(len(systems)))
    ax1.set_yticklabels(systems, fontsize=8)
    ax1.invert_yaxis()
    ax1.set_xlabel('Average Score (Higher = Easier)')
    ax1.set_title('Simulation Systems by Difficulty', fontsize=12, fontweight='bold')
    ax1.axvline(x=scores.mean(), color='red', linestyle='--', alpha=0.7, label=f'Mean: {scores.mean():.1f}')
    ax1.legend()
    
    # 2. 类别对比箱线图
    ax2 = axes[0, 1]
    category_order = category_stats.sort_values('mean', ascending=False).index.tolist()
    sns.boxplot(data=df, x='Category', y='Score_Avg', order=category_order, ax=ax2, palette='Set2')
    ax2.set_xlabel('System Category')
    ax2.set_ylabel('Average Score')
    ax2.set_title('Performance by System Category', fontsize=12, fontweight='bold')
    
    # 3. 系统热力图 (模型 x 系统)
    ax3 = axes[1, 0]
    # 选择Top 10模型和所有系统
    top10_models = df.groupby('Model')['Score_Avg'].mean().sort_values(ascending=False).head(10).index
    heatmap_data = df[df['Model'].isin(top10_models)].pivot_table(
        values='Score_Avg', index='Model', columns='System', aggfunc='mean'
    )
    # 按系统难度排序
    heatmap_data = heatmap_data[system_stats.index]
    sns.heatmap(heatmap_data, cmap='RdYlGn', ax=ax3, cbar_kws={'label': 'Score'})
    ax3.set_title('Top 10 Models × All Systems Heatmap', fontsize=12, fontweight='bold')
    ax3.set_xticklabels(ax3.get_xticklabels(), rotation=45, ha='right', fontsize=7)
    
    # 4. 模型在不同类别上的表现差异
    ax4 = axes[1, 1]
    # Top 5 模型在各类别的表现
    top5_models = df.groupby('Model')['Score_Avg'].mean().sort_values(ascending=False).head(5).index
    df_top5 = df[df['Model'].isin(top5_models)]
    category_pivot = df_top5.pivot_table(values='Score_Avg', index='Model', columns='Category', aggfunc='mean')
    category_pivot.plot(kind='bar', ax=ax4, width=0.8)
    ax4.set_ylabel('Average Score')
    ax4.set_title('Top 5 Models Performance by Category', fontsize=12, fontweight='bold')
    ax4.legend(title='Category', bbox_to_anchor=(1.05, 1))
    ax4.set_xticklabels(ax4.get_xticklabels(), rotation=45, ha='right')
    
    plt.tight_layout()
    plt.savefig(OUTPUT_DIR / "03_task_difficulty.png", bbox_inches='tight')
    plt.close()
    
    print(f"\n✅ 结果已保存到 {OUTPUT_DIR}/03_task_difficulty.csv 和 .png")
    
    return system_stats, category_stats

def analyze_score_correlation(df):
    """
    分析4: 评分维度相关性分析
    """
    print("\n" + "="*60)
    print("📊 分析4: 评分维度相关性分析")
    print("="*60)
    
    # 计算相关系数
    score_cols = ['Score_Doc', 'Score_Ref', 'Score_RefDoc']
    corr_matrix = df[score_cols].corr()
    
    print("\n📈 Pearson 相关系数矩阵:")
    print("-" * 50)
    print(corr_matrix.round(3).to_string())
    
    # Spearman 相关
    spearman_corr = df[score_cols].corr(method='spearman')
    print("\n📈 Spearman 相关系数矩阵:")
    print("-" * 50)
    print(spearman_corr.round(3).to_string())
    
    # 保存
    corr_matrix.to_csv(OUTPUT_DIR / "04_correlation_pearson.csv")
    spearman_corr.to_csv(OUTPUT_DIR / "04_correlation_spearman.csv")
    
    # 可视化
    fig, axes = plt.subplots(2, 2, figsize=(14, 12))
    
    # 1. Pearson 相关性热力图
    ax1 = axes[0, 0]
    sns.heatmap(corr_matrix, annot=True, cmap='RdBu_r', center=0, 
                vmin=-1, vmax=1, ax=ax1, square=True, fmt='.3f',
                annot_kws={'fontsize': 14, 'fontweight': 'bold'})
    ax1.set_title('Pearson Correlation Matrix', fontsize=12, fontweight='bold')
    
    # 2. Spearman 相关性热力图
    ax2 = axes[0, 1]
    sns.heatmap(spearman_corr, annot=True, cmap='RdBu_r', center=0,
                vmin=-1, vmax=1, ax=ax2, square=True, fmt='.3f',
                annot_kws={'fontsize': 14, 'fontweight': 'bold'})
    ax2.set_title('Spearman Correlation Matrix', fontsize=12, fontweight='bold')
    
    # 3. 散点图矩阵
    ax3 = axes[1, 0]
    ax3.scatter(df['Score_Doc'], df['Score_Ref'], alpha=0.3, s=10)
    ax3.set_xlabel('Score Document')
    ax3.set_ylabel('Score Reference')
    ax3.set_title(f'Score Doc vs Ref (r={corr_matrix.loc["Score_Doc", "Score_Ref"]:.3f})', 
                  fontsize=12, fontweight='bold')
    # 添加回归线
    z = np.polyfit(df['Score_Doc'], df['Score_Ref'], 1)
    p = np.poly1d(z)
    ax3.plot(df['Score_Doc'].sort_values(), p(df['Score_Doc'].sort_values()), 
             "r--", alpha=0.8, linewidth=2)
    
    # 4. Score_Ref vs Score_RefDoc
    ax4 = axes[1, 1]
    ax4.scatter(df['Score_Ref'], df['Score_RefDoc'], alpha=0.3, s=10, c='green')
    ax4.set_xlabel('Score Reference')
    ax4.set_ylabel('Score Reference Document')
    ax4.set_title(f'Score Ref vs RefDoc (r={corr_matrix.loc["Score_Ref", "Score_RefDoc"]:.3f})', 
                  fontsize=12, fontweight='bold')
    z = np.polyfit(df['Score_Ref'], df['Score_RefDoc'], 1)
    p = np.poly1d(z)
    ax4.plot(df['Score_Ref'].sort_values(), p(df['Score_Ref'].sort_values()), 
             "r--", alpha=0.8, linewidth=2)
    
    plt.tight_layout()
    plt.savefig(OUTPUT_DIR / "04_correlation.png", bbox_inches='tight')
    plt.close()
    
    print(f"\n✅ 结果已保存到 {OUTPUT_DIR}/04_correlation_*.csv 和 .png")
    
    return corr_matrix

def analyze_model_families(df):
    """
    分析5: 模型家族对比分析
    """
    print("\n" + "="*60)
    print("📊 分析5: 模型家族对比分析")
    print("="*60)
    
    # 定义模型家族
    def get_model_family(model_name):
        model_lower = model_name.lower()
        if 'claude' in model_lower:
            return 'Claude'
        elif 'gpt' in model_lower or model_lower.startswith('o3') or model_lower.startswith('o4'):
            return 'OpenAI'
        elif 'deepseek' in model_lower:
            return 'DeepSeek'
        elif 'llama' in model_lower:
            return 'Llama'
        elif 'gemma' in model_lower:
            return 'Gemma'
        elif 'qwen' in model_lower:
            return 'Qwen'
        elif 'mistral' in model_lower:
            return 'Mistral'
        elif 'phi' in model_lower:
            return 'Phi'
        elif 'nemotron' in model_lower:
            return 'Nemotron'
        else:
            return 'Other'
    
    df['Family'] = df['Model'].apply(get_model_family)
    
    # 家族统计
    family_stats = df.groupby('Family').agg({
        'Score_Avg': ['mean', 'std', 'count'],
        'Score_Doc': 'mean',
        'Score_Ref': 'mean',
        'Score_RefDoc': 'mean'
    }).round(2)
    family_stats.columns = ['_'.join(col).strip() for col in family_stats.columns.values]
    family_stats = family_stats.sort_values('Score_Avg_mean', ascending=False)
    
    print("\n🏢 模型家族排名:")
    print("-" * 80)
    print(family_stats.to_string())
    
    # 保存
    family_stats.to_csv(OUTPUT_DIR / "05_model_families.csv")
    
    # 可视化
    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    
    # 1. 家族平均得分
    ax1 = axes[0, 0]
    families = family_stats.index.tolist()
    scores = family_stats['Score_Avg_mean'].values
    colors = plt.cm.Set3(np.linspace(0, 1, len(families)))
    bars = ax1.barh(range(len(families)), scores, color=colors)
    ax1.set_yticks(range(len(families)))
    ax1.set_yticklabels(families)
    ax1.invert_yaxis()
    ax1.set_xlabel('Average Score')
    ax1.set_title('Model Family Ranking', fontsize=12, fontweight='bold')
    
    # 添加数值标签
    for bar, val in zip(bars, scores):
        ax1.text(val + 1, bar.get_y() + bar.get_height()/2, f'{val:.1f}', 
                va='center', fontsize=10)
    
    # 2. 家族箱线图
    ax2 = axes[0, 1]
    family_order = family_stats.index.tolist()
    sns.boxplot(data=df, x='Family', y='Score_Avg', order=family_order, ax=ax2, palette='Set3')
    ax2.set_xlabel('Model Family')
    ax2.set_ylabel('Average Score')
    ax2.set_title('Score Distribution by Family', fontsize=12, fontweight='bold')
    ax2.set_xticklabels(ax2.get_xticklabels(), rotation=45, ha='right')
    
    # 3. 家族在三个维度的对比
    ax3 = axes[1, 0]
    x = np.arange(len(families))
    width = 0.25
    ax3.bar(x - width, family_stats['Score_Doc_mean'], width, label='Score Doc', color='#3498db')
    ax3.bar(x, family_stats['Score_Ref_mean'], width, label='Score Ref', color='#2ecc71')
    ax3.bar(x + width, family_stats['Score_RefDoc_mean'], width, label='Score RefDoc', color='#e74c3c')
    ax3.set_xticks(x)
    ax3.set_xticklabels(families, rotation=45, ha='right')
    ax3.set_ylabel('Score')
    ax3.set_title('Three Dimensions by Family', fontsize=12, fontweight='bold')
    ax3.legend()
    
    # 4. 商业 vs 开源对比
    ax4 = axes[1, 1]
    commercial = ['Claude', 'OpenAI', 'Mistral']
    opensource = ['DeepSeek', 'Llama', 'Gemma', 'Qwen', 'Phi']
    
    df['Type'] = df['Family'].apply(lambda x: 'Commercial' if x in commercial else 'Open Source')
    type_stats = df.groupby('Type')['Score_Avg'].agg(['mean', 'std'])
    
    bars = ax4.bar(type_stats.index, type_stats['mean'], yerr=type_stats['std'], 
                   color=['#e74c3c', '#3498db'], capsize=10, alpha=0.8)
    ax4.set_ylabel('Average Score')
    ax4.set_title('Commercial vs Open Source', fontsize=12, fontweight='bold')
    
    for bar, val in zip(bars, type_stats['mean']):
        ax4.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 2, 
                f'{val:.1f}', ha='center', fontsize=14, fontweight='bold')
    
    plt.tight_layout()
    plt.savefig(OUTPUT_DIR / "05_model_families.png", bbox_inches='tight')
    plt.close()
    
    print(f"\n✅ 结果已保存到 {OUTPUT_DIR}/05_model_families.csv 和 .png")
    
    return family_stats

def generate_radar_chart(df):
    """
    分析6: 生成雷达图 - Top 5 模型能力雷达
    """
    print("\n" + "="*60)
    print("📊 分析6: 生成能力雷达图")
    print("="*60)
    
    # 获取Top 5模型
    model_avg = df.groupby('Model')['Score_Avg'].mean().sort_values(ascending=False)
    top5_models = model_avg.head(5).index.tolist()
    
    # 定义系统类别 (基于官方分类)
    categories = {
        'MBS': ['pendulum', 'slider_crank', 'gear', 'mass_spring_damper', 'particles'],
        'FEA': ['beam', 'buckling', 'rotor', 'tablecloth', 'cable'],
        'Sensor': ['gps_imu', 'lidar', 'veh_app', 'camera'],
        'Robot': ['turtlebot', 'viper', 'curiosity', 'vehros', 'sensros', 'handler'],
        'Vehicle': ['citybus', 'feda', 'gator', 'hmmwv', 'kraz', 'art', 'rigid_highway', 
                   'rigid_multipatches', 'scm', 'scm_hill', 'uazbus', 'm113', 'sedan', 'man']
    }
    
    # 计算每个模型在每个类别的平均分
    radar_data = {}
    for model in top5_models:
        model_df = df[df['Model'] == model]
        cat_scores = {}
        for cat, systems in categories.items():
            cat_df = model_df[model_df['System'].isin(systems)]
            if len(cat_df) > 0:
                cat_scores[cat] = cat_df['Score_Avg'].mean()
            else:
                cat_scores[cat] = 0
        radar_data[model] = cat_scores
    
    radar_df = pd.DataFrame(radar_data)
    radar_df.to_csv(OUTPUT_DIR / "06_radar_data.csv")
    
    # 创建雷达图
    categories_list = list(categories.keys())
    num_cats = len(categories_list)
    
    angles = np.linspace(0, 2 * np.pi, num_cats, endpoint=False).tolist()
    angles += angles[:1]  # 闭合
    
    fig, ax = plt.subplots(figsize=(10, 10), subplot_kw=dict(polar=True))
    
    colors = plt.cm.Set1(np.linspace(0, 1, len(top5_models)))
    
    for idx, model in enumerate(top5_models):
        values = [radar_data[model][cat] for cat in categories_list]
        values += values[:1]  # 闭合
        
        ax.plot(angles, values, 'o-', linewidth=2, label=model, color=colors[idx])
        ax.fill(angles, values, alpha=0.1, color=colors[idx])
    
    ax.set_xticks(angles[:-1])
    ax.set_xticklabels(categories_list, fontsize=12)
    ax.set_ylim(0, 100)
    ax.set_title('Top 5 Models Capability Radar', fontsize=14, fontweight='bold', pad=20)
    ax.legend(loc='upper right', bbox_to_anchor=(1.3, 1.1))
    
    plt.tight_layout()
    plt.savefig(OUTPUT_DIR / "06_radar_chart.png", bbox_inches='tight')
    plt.close()
    
    print(f"\n✅ 雷达图已保存到 {OUTPUT_DIR}/06_radar_chart.png")
    
    return radar_df

def statistical_tests(df):
    """
    分析7: 统计显著性检验
    """
    print("\n" + "="*60)
    print("📊 分析7: 统计显著性检验")
    print("="*60)
    
    # 获取Top 3 模型
    model_avg = df.groupby('Model')['Score_Avg'].mean().sort_values(ascending=False)
    top3 = model_avg.head(3).index.tolist()
    
    print(f"\n🔬 Top 3 模型配对检验: {top3}")
    print("-" * 60)
    
    results = []
    
    # 配对 t 检验
    for i in range(len(top3)):
        for j in range(i + 1, len(top3)):
            model1, model2 = top3[i], top3[j]
            scores1 = df[df['Model'] == model1]['Score_Avg'].values
            scores2 = df[df['Model'] == model2]['Score_Avg'].values
            
            # 确保长度相同（取最小长度）
            min_len = min(len(scores1), len(scores2))
            scores1 = scores1[:min_len]
            scores2 = scores2[:min_len]
            
            # t-test
            t_stat, t_pval = stats.ttest_ind(scores1, scores2)
            
            # Wilcoxon
            try:
                w_stat, w_pval = stats.wilcoxon(scores1, scores2)
            except:
                w_stat, w_pval = np.nan, np.nan
            
            # 效应量 (Cohen's d)
            pooled_std = np.sqrt((np.std(scores1)**2 + np.std(scores2)**2) / 2)
            cohens_d = (np.mean(scores1) - np.mean(scores2)) / pooled_std if pooled_std > 0 else 0
            
            result = {
                'Model 1': model1,
                'Model 2': model2,
                'Mean 1': np.mean(scores1),
                'Mean 2': np.mean(scores2),
                't-statistic': t_stat,
                't p-value': t_pval,
                'Wilcoxon p-value': w_pval,
                "Cohen's d": cohens_d,
                'Significant (p<0.05)': 'Yes' if t_pval < 0.05 else 'No'
            }
            results.append(result)
            
            print(f"\n{model1} vs {model2}:")
            print(f"  Mean: {np.mean(scores1):.2f} vs {np.mean(scores2):.2f}")
            print(f"  t-test p-value: {t_pval:.4f} {'*' if t_pval < 0.05 else ''}")
            print(f"  Cohen's d: {cohens_d:.3f}")
    
    results_df = pd.DataFrame(results)
    results_df.to_csv(OUTPUT_DIR / "07_statistical_tests.csv", index=False)
    
    # ANOVA - 检验模型家族是否有显著差异
    print("\n\n🔬 ANOVA - 模型家族差异检验:")
    print("-" * 60)
    
    def get_family(model):
        model_lower = model.lower()
        if 'claude' in model_lower:
            return 'Claude'
        elif 'gpt' in model_lower or model_lower.startswith('o3') or model_lower.startswith('o4'):
            return 'OpenAI'
        elif 'deepseek' in model_lower:
            return 'DeepSeek'
        elif 'llama' in model_lower:
            return 'Llama'
        elif 'gemma' in model_lower:
            return 'Gemma'
        elif 'qwen' in model_lower:
            return 'Qwen'
        else:
            return 'Other'
    
    df['Family'] = df['Model'].apply(get_family)
    families = df['Family'].unique()
    family_scores = [df[df['Family'] == f]['Score_Avg'].values for f in families]
    
    f_stat, anova_pval = stats.f_oneway(*family_scores)
    print(f"  F-statistic: {f_stat:.3f}")
    print(f"  p-value: {anova_pval:.6f}")
    print(f"  结论: {'模型家族间存在显著差异' if anova_pval < 0.05 else '模型家族间无显著差异'}")
    
    print(f"\n✅ 结果已保存到 {OUTPUT_DIR}/07_statistical_tests.csv")
    
    return results_df

def generate_summary_table(df):
    """
    生成论文用的汇总表格
    """
    print("\n" + "="*60)
    print("📊 生成论文汇总表格")
    print("="*60)
    
    # 主排名表
    model_stats = df.groupby('Model').agg({
        'Score_Avg': ['mean', 'std'],
        'Score_Doc': 'mean',
        'Score_Ref': 'mean',
        'Score_RefDoc': 'mean'
    }).round(2)
    
    model_stats.columns = ['Avg Score', 'Std', 'Doc Score', 'Ref Score', 'RefDoc Score']
    model_stats = model_stats.sort_values('Avg Score', ascending=False)
    model_stats['Rank'] = range(1, len(model_stats) + 1)
    model_stats = model_stats[['Rank', 'Avg Score', 'Std', 'Doc Score', 'Ref Score', 'RefDoc Score']]
    
    # 保存 LaTeX 格式
    latex_table = model_stats.to_latex(index=True, float_format="%.2f")
    with open(OUTPUT_DIR / "paper_table_ranking.tex", 'w') as f:
        f.write(latex_table)
    
    # 保存 CSV
    model_stats.to_csv(OUTPUT_DIR / "paper_table_ranking.csv")
    
    print("\n📋 论文主表格 (模型排名):")
    print("-" * 80)
    print(model_stats.head(15).to_string())
    
    print(f"\n✅ LaTeX表格已保存到 {OUTPUT_DIR}/paper_table_ranking.tex")
    
    return model_stats

def main():
    """主函数"""
    print("\n" + "="*70)
    print("🚀 SimBench LLM Benchmark 数据分析")
    print("="*70)
    
    # 加载数据
    df = load_data()
    
    # 执行各项分析
    model_stats = analyze_model_ranking(df)
    round_stats = analyze_round_progression(df)
    system_stats, category_stats = analyze_task_difficulty(df)
    corr_matrix = analyze_score_correlation(df)
    family_stats = analyze_model_families(df)
    radar_df = generate_radar_chart(df)
    stat_results = statistical_tests(df)
    summary_table = generate_summary_table(df)
    
    print("\n" + "="*70)
    print("✅ 所有分析完成!")
    print(f"📁 结果已保存到: {OUTPUT_DIR}")
    print("="*70)
    
    # 列出生成的文件
    print("\n📄 生成的文件:")
    for f in sorted(OUTPUT_DIR.glob("*")):
        print(f"   - {f.name}")

if __name__ == "__main__":
    main()
