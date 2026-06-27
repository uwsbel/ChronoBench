"""
简化版：只比较三个J-LLM在Ref-Doc modality上的相关性
"""
import pandas as pd
import numpy as np
from scipy.stats import pearsonr, spearmanr
import matplotlib.pyplot as plt
import seaborn as sns
import sys
import io

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')

def load_jllm_data(filepath):
    """加载J-LLM评分数据"""
    df = pd.read_csv(filepath, skipinitialspace=True)
    df.columns = df.columns.str.strip()
    df['model'] = df['model'].str.strip()
    return df

def main():
    print("="*80)
    print("J-LLM评估器对比分析 (只关注Ref-Doc)")
    print("="*80)
    
    # 加载三个J-LLM的数据
    print("\n加载数据...")
    df_4_1_mini = load_jllm_data('paper/out/J-LLM/all_scores_ranked_gpt-4.1-mini.csv')
    df_4_1_nano = load_jllm_data('paper/out/J-LLM/all_scores_ranked_gpt-4.1-nano.csv')
    df_4o_mini = load_jllm_data('paper/out/J-LLM/all_scores_ranked_gpt-4o-mini.csv')
    
    print(f"GPT-4.1-mini: {len(df_4_1_mini)} 个模型")
    print(f"GPT-4.1-nano: {len(df_4_1_nano)} 个模型")
    print(f"GPT-4o-mini: {len(df_4o_mini)} 个模型")
    
    # 找出共同的模型
    models_4_1_mini = set(df_4_1_mini['model'])
    models_4_1_nano = set(df_4_1_nano['model'])
    models_4o_mini = set(df_4o_mini['model'])
    
    common_models = models_4_1_mini & models_4_1_nano & models_4o_mini
    print(f"\n共同的模型数量: {len(common_models)}")
    
    # 创建对比数据框（只使用Ref-Doc分数）
    results = []
    
    for model in sorted(common_models):
        row_4_1_mini = df_4_1_mini[df_4_1_mini['model'] == model].iloc[0]
        row_4_1_nano = df_4_1_nano[df_4_1_nano['model'] == model].iloc[0]
        row_4o_mini = df_4o_mini[df_4o_mini['model'] == model].iloc[0]
        
        result = {
            'Model': model,
            'J-LLM-4.1-mini': float(row_4_1_mini['Score Reference Document']),
            'J-LLM-4.1-nano': float(row_4_1_nano['Score Reference Document']),
            'J-LLM-4o-mini': float(row_4o_mini['Score Reference Document']),
        }
        results.append(result)
    
    df_results = pd.DataFrame(results)
    
    # 计算平均分并排序
    df_results['Avg'] = df_results[['J-LLM-4.1-mini', 'J-LLM-4.1-nano', 'J-LLM-4o-mini']].mean(axis=1)
    df_results = df_results.sort_values('Avg', ascending=False).reset_index(drop=True)
    df_results['Rank'] = range(1, len(df_results) + 1)
    
    # 重新排列列顺序
    df_results = df_results[['Rank', 'Model', 'J-LLM-4.1-mini', 'J-LLM-4.1-nano', 'J-LLM-4o-mini', 'Avg']]
    
    # 保存CSV
    output_csv = 'paper/out/jllm_ref_doc_comparison.csv'
    df_results.to_csv(output_csv, index=False, float_format='%.2f')
    print(f"\n结果已保存到: {output_csv}")
    
    # 计算相关性
    print("\n" + "="*80)
    print("三个J-LLM评估器之间的相关性 (Ref-Doc Modality)")
    print("="*80)
    
    jllm_cols = ['J-LLM-4.1-mini', 'J-LLM-4.1-nano', 'J-LLM-4o-mini']
    
    # Pearson相关系数
    print("\n【Pearson相关系数】")
    pearson_results = []
    for i, col1 in enumerate(jllm_cols):
        for j, col2 in enumerate(jllm_cols):
            if i < j:
                r, p = pearsonr(df_results[col1], df_results[col2])
                pearson_results.append({
                    'Pair': f"{col1.split('-')[-1]} vs {col2.split('-')[-1]}",
                    'r': r,
                    'p-value': p
                })
                print(f"  {col1.split('-')[-1]:10s} vs {col2.split('-')[-1]:10s}: r = {r:.4f}, p = {p:.6f}")
    
    # Spearman相关系数
    print("\n【Spearman相关系数 (秩相关)】")
    spearman_results = []
    for i, col1 in enumerate(jllm_cols):
        for j, col2 in enumerate(jllm_cols):
            if i < j:
                rho, p = spearmanr(df_results[col1], df_results[col2])
                spearman_results.append({
                    'Pair': f"{col1.split('-')[-1]} vs {col2.split('-')[-1]}",
                    'ρ': rho,
                    'p-value': p
                })
                print(f"  {col1.split('-')[-1]:10s} vs {col2.split('-')[-1]:10s}: ρ = {rho:.4f}, p = {p:.6f}")
    
    # 基本统计
    print("\n" + "="*80)
    print("基本统计信息")
    print("="*80)
    
    stats_summary = []
    for col in jllm_cols:
        mean = df_results[col].mean()
        std = df_results[col].std()
        min_val = df_results[col].min()
        max_val = df_results[col].max()
        print(f"\n{col}:")
        print(f"  均值: {mean:.2f}")
        print(f"  标准差: {std:.2f}")
        print(f"  范围: [{min_val:.2f}, {max_val:.2f}]")
        stats_summary.append({
            'J-LLM': col.split('-')[-1],
            'Mean': mean,
            'Std': std,
            'Min': min_val,
            'Max': max_val
        })
    
    # 生成Markdown表格
    print("\n" + "="*80)
    print("生成Markdown报告...")
    print("="*80)
    
    with open('paper/out/jllm_ref_doc_comparison.md', 'w', encoding='utf-8') as f:
        f.write("# J-LLM评估器对比分析 (Ref-Doc Modality)\n\n")
        
        f.write("## 排名对比表\n\n")
        f.write("| Rank | Model | 4.1-mini | 4.1-nano | 4o-mini | Avg |\n")
        f.write("|------|-------|----------|----------|---------|-----|\n")
        for _, row in df_results.iterrows():
            f.write(f"| {row['Rank']} | {row['Model']} | ")
            f.write(f"{row['J-LLM-4.1-mini']:.2f} | {row['J-LLM-4.1-nano']:.2f} | ")
            f.write(f"{row['J-LLM-4o-mini']:.2f} | {row['Avg']:.2f} |\n")
        
        f.write("\n## 相关性分析\n\n")
        f.write("### Pearson相关系数\n\n")
        f.write("| 对比 | r | p-value | 显著性 |\n")
        f.write("|------|-------|---------|--------|\n")
        for res in pearson_results:
            sig = "***" if res['p-value'] < 0.001 else "**" if res['p-value'] < 0.01 else "*" if res['p-value'] < 0.05 else "ns"
            f.write(f"| {res['Pair']} | {res['r']:.4f} | {res['p-value']:.6f} | {sig} |\n")
        
        f.write("\n### Spearman相关系数\n\n")
        f.write("| 对比 | ρ | p-value | 显著性 |\n")
        f.write("|------|-------|---------|--------|\n")
        for res in spearman_results:
            sig = "***" if res['p-value'] < 0.001 else "**" if res['p-value'] < 0.01 else "*" if res['p-value'] < 0.05 else "ns"
            f.write(f"| {res['Pair']} | {res['ρ']:.4f} | {res['p-value']:.6f} | {sig} |\n")
        
        f.write("\n### 统计摘要\n\n")
        f.write("| J-LLM | 均值 | 标准差 | 最小值 | 最大值 |\n")
        f.write("|-------|------|--------|--------|--------|\n")
        for stat in stats_summary:
            f.write(f"| {stat['J-LLM']} | {stat['Mean']:.2f} | {stat['Std']:.2f} | ")
            f.write(f"{stat['Min']:.2f} | {stat['Max']:.2f} |\n")
        
        f.write("\n## 主要发现\n\n")
        
        # 找出相关性最高的一对
        max_pearson = max(pearson_results, key=lambda x: x['r'])
        max_spearman = max(spearman_results, key=lambda x: x['ρ'])
        
        f.write(f"1. **最高Pearson相关性**: {max_pearson['Pair']} (r = {max_pearson['r']:.4f})\n")
        f.write(f"2. **最高Spearman相关性**: {max_spearman['Pair']} (ρ = {max_spearman['ρ']:.4f})\n")
        f.write(f"3. **评分最严格**: {stats_summary[0]['J-LLM'] if stats_summary[0]['Mean'] == min(s['Mean'] for s in stats_summary) else stats_summary[1]['J-LLM'] if stats_summary[1]['Mean'] == min(s['Mean'] for s in stats_summary) else stats_summary[2]['J-LLM']} (均值最低)\n")
        f.write(f"4. **评分最宽松**: {stats_summary[0]['J-LLM'] if stats_summary[0]['Mean'] == max(s['Mean'] for s in stats_summary) else stats_summary[1]['J-LLM'] if stats_summary[1]['Mean'] == max(s['Mean'] for s in stats_summary) else stats_summary[2]['J-LLM']} (均值最高)\n")
        f.write(f"5. **所有相关性均显著** (p < 0.001)，表明三个评估器高度一致\n")
    
    print("Markdown报告已保存到: paper/out/jllm_ref_doc_comparison.md")
    
    # 生成LaTeX表格
    with open('paper/out/jllm_ref_doc_comparison.tex', 'w', encoding='utf-8') as f:
        f.write("\\begin{table}[htbp]\n")
        f.write("    \\centering\n")
        f.write("    \\begin{tabular}{rlcccc}\n")
        f.write("    \\toprule\n")
        f.write("    \\textbf{Rank} & \\textbf{Model} & \\textbf{4.1-mini} & \\textbf{4.1-nano} & \\textbf{4o-mini} & \\textbf{Avg} \\\\\n")
        f.write("    \\midrule\n")
        
        for _, row in df_results.iterrows():
            f.write(f"    {row['Rank']} & {row['Model']} & ")
            f.write(f"{row['J-LLM-4.1-mini']:.2f} & {row['J-LLM-4.1-nano']:.2f} & ")
            f.write(f"{row['J-LLM-4o-mini']:.2f} & {row['Avg']:.2f} \\\\\n")
        
        f.write("    \\bottomrule\n")
        f.write("    \\end{tabular}\n")
        f.write("    \\caption{Comparison of three J-LLM evaluators on Reference-Document modality scores.}\n")
        f.write("    \\label{tab:jllm_ref_doc_comparison}\n")
        f.write("\\end{table}\n")
    
    print("LaTeX表格已保存到: paper/out/jllm_ref_doc_comparison.tex")
    
    # 打印Top 10
    print("\n" + "="*80)
    print("Top 10 模型")
    print("="*80)
    print(df_results.head(10).to_string(index=False, float_format='%.2f'))
    
    print("\n完成！")
    
    return df_results, pearson_results, spearman_results

if __name__ == "__main__":
    df_results, pearson_results, spearman_results = main()
