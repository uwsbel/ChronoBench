"""
比较三个J-LLM评估器的结果
"""
import pandas as pd
import numpy as np
from scipy.stats import pearsonr, spearmanr
import sys
import io

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')

def load_jllm_data(filepath):
    """加载J-LLM评分数据"""
    df = pd.read_csv(filepath, skipinitialspace=True)
    # 清理列名
    df.columns = df.columns.str.strip()
    return df

def main():
    # 加载三个J-LLM的数据
    print("加载数据...")
    df_4_1_mini = load_jllm_data('paper/out/J-LLM/all_scores_ranked_gpt-4.1-mini.csv')
    df_4_1_nano = load_jllm_data('paper/out/J-LLM/all_scores_ranked_gpt-4.1-nano.csv')
    df_4o_mini = load_jllm_data('paper/out/J-LLM/all_scores_ranked_gpt-4o-mini.csv')
    
    print(f"GPT-4.1-mini: {len(df_4_1_mini)} 个模型")
    print(f"GPT-4.1-nano: {len(df_4_1_nano)} 个模型")
    print(f"GPT-4o-mini: {len(df_4o_mini)} 个模型")
    
    # 清理模型名称
    for df in [df_4_1_mini, df_4_1_nano, df_4o_mini]:
        df['model'] = df['model'].str.strip()
    
    # 找出共同的模型
    models_4_1_mini = set(df_4_1_mini['model'])
    models_4_1_nano = set(df_4_1_nano['model'])
    models_4o_mini = set(df_4o_mini['model'])
    
    common_models = models_4_1_mini & models_4_1_nano & models_4o_mini
    print(f"\n共同的base model数量: {len(common_models)}")
    
    # 创建合并数据框
    results = []
    
    for model in sorted(common_models):
        row_4_1_mini = df_4_1_mini[df_4_1_mini['model'] == model].iloc[0]
        row_4_1_nano = df_4_1_nano[df_4_1_nano['model'] == model].iloc[0]
        row_4o_mini = df_4o_mini[df_4o_mini['model'] == model].iloc[0]
        
        result = {
            'Model': model,
            # 三个J-LLM的Ref-Doc分数
            'J-LLM-4.1-mini': row_4_1_mini['Score Reference Document'],
            'J-LLM-4.1-nano': row_4_1_nano['Score Reference Document'],
            'J-LLM-4o-mini': row_4o_mini['Score Reference Document'],
            # 三个modality（使用4.1-mini作为参考）
            'Ref-Doc': row_4_1_mini['Score Reference Document'],
            'Ref': row_4_1_mini['Score Reference'],
            'Doc': row_4_1_mini['Score Document'],
        }
        results.append(result)
    
    df_results = pd.DataFrame(results)
    
    # 计算平均J-LLM分数并排序
    df_results['Avg-J-LLM'] = df_results[['J-LLM-4.1-mini', 'J-LLM-4.1-nano', 'J-LLM-4o-mini']].mean(axis=1)
    df_results = df_results.sort_values('Avg-J-LLM', ascending=False).reset_index(drop=True)
    df_results['Rank'] = range(1, len(df_results) + 1)
    
    # 重新排列列顺序
    df_results = df_results[['Rank', 'Model', 'J-LLM-4.1-mini', 'J-LLM-4.1-nano', 'J-LLM-4o-mini', 
                             'Avg-J-LLM', 'Ref-Doc', 'Ref', 'Doc']]
    
    # 保存CSV
    output_csv = 'paper/out/jllm_comparison.csv'
    df_results.to_csv(output_csv, index=False, float_format='%.2f')
    print(f"\n结果已保存到: {output_csv}")
    
    # 计算J-LLM之间的相关性
    print("\n" + "="*80)
    print("J-LLM评估器之间的相关性分析")
    print("="*80)
    
    jllm_cols = ['J-LLM-4.1-mini', 'J-LLM-4.1-nano', 'J-LLM-4o-mini']
    
    print("\nPearson相关系数:")
    pearson_matrix = []
    for i, col1 in enumerate(jllm_cols):
        row = []
        for j, col2 in enumerate(jllm_cols):
            r, p = pearsonr(df_results[col1], df_results[col2])
            row.append(r)
            if i < j:  # 只打印上三角
                print(f"  {col1} vs {col2}: r = {r:.4f}, p = {p:.6f}")
        pearson_matrix.append(row)
    
    print("\nSpearman相关系数 (秩相关):")
    spearman_matrix = []
    for i, col1 in enumerate(jllm_cols):
        row = []
        for j, col2 in enumerate(jllm_cols):
            rho, p = spearmanr(df_results[col1], df_results[col2])
            row.append(rho)
            if i < j:  # 只打印上三角
                print(f"  {col1} vs {col2}: ρ = {rho:.4f}, p = {p:.6f}")
        spearman_matrix.append(row)
    
    # 计算modality之间的相关性
    print("\n" + "="*80)
    print("不同Modality之间的相关性")
    print("="*80)
    
    modality_cols = ['Ref-Doc', 'Ref', 'Doc']
    
    print("\nPearson相关系数:")
    for i, col1 in enumerate(modality_cols):
        for j, col2 in enumerate(modality_cols):
            if i < j:
                r, p = pearsonr(df_results[col1], df_results[col2])
                print(f"  {col1} vs {col2}: r = {r:.4f}, p = {p:.6f}")
    
    print("\n" + "="*80)
    print("基本统计信息")
    print("="*80)
    
    for col in jllm_cols:
        mean = df_results[col].mean()
        std = df_results[col].std()
        min_val = df_results[col].min()
        max_val = df_results[col].max()
        print(f"\n{col}:")
        print(f"  Mean: {mean:.2f}, Std: {std:.2f}")
        print(f"  Range: [{min_val:.2f}, {max_val:.2f}]")
    
    # 生成Markdown表格
    print("\n" + "="*80)
    print("生成Markdown表格...")
    print("="*80)
    
    with open('paper/out/jllm_comparison.md', 'w', encoding='utf-8') as f:
        f.write("# J-LLM评估器对比分析\n\n")
        f.write("## 评分对比表\n\n")
        
        # 表头
        f.write("| Rank | Model | J-LLM-4.1-mini | J-LLM-4.1-nano | J-LLM-4o-mini | Avg | Ref-Doc | Ref | Doc |\n")
        f.write("|------|-------|----------------|----------------|---------------|-----|---------|-----|-----|\n")
        
        # 数据行
        for _, row in df_results.iterrows():
            f.write(f"| {row['Rank']} | {row['Model']} | ")
            f.write(f"{row['J-LLM-4.1-mini']:.2f} | {row['J-LLM-4.1-nano']:.2f} | {row['J-LLM-4o-mini']:.2f} | ")
            f.write(f"{row['Avg-J-LLM']:.2f} | {row['Ref-Doc']:.2f} | {row['Ref']:.2f} | {row['Doc']:.2f} |\n")
        
        f.write("\n## J-LLM评估器相关性\n\n")
        f.write("### Pearson相关系数\n\n")
        f.write("| | J-LLM-4.1-mini | J-LLM-4.1-nano | J-LLM-4o-mini |\n")
        f.write("|---|----------------|----------------|---------------|\n")
        for i, col1 in enumerate(jllm_cols):
            f.write(f"| {col1} |")
            for j, col2 in enumerate(jllm_cols):
                r, _ = pearsonr(df_results[col1], df_results[col2])
                f.write(f" {r:.4f} |")
            f.write("\n")
        
        f.write("\n### Spearman相关系数 (秩相关)\n\n")
        f.write("| | J-LLM-4.1-mini | J-LLM-4.1-nano | J-LLM-4o-mini |\n")
        f.write("|---|----------------|----------------|---------------|\n")
        for i, col1 in enumerate(jllm_cols):
            f.write(f"| {col1} |")
            for j, col2 in enumerate(jllm_cols):
                rho, _ = spearmanr(df_results[col1], df_results[col2])
                f.write(f" {rho:.4f} |")
            f.write("\n")
        
        f.write("\n## Modality相关性\n\n")
        f.write("### Pearson相关系数\n\n")
        f.write("| | Ref-Doc | Ref | Doc |\n")
        f.write("|---|---------|-----|-----|\n")
        for col1 in modality_cols:
            f.write(f"| {col1} |")
            for col2 in modality_cols:
                r, _ = pearsonr(df_results[col1], df_results[col2])
                f.write(f" {r:.4f} |")
            f.write("\n")
    
    print("Markdown表格已保存到: paper/out/jllm_comparison.md")
    
    # 生成LaTeX表格
    print("\n生成LaTeX表格...")
    
    with open('paper/out/jllm_comparison.tex', 'w', encoding='utf-8') as f:
        f.write("\\begin{table*}[htbp]\n")
        f.write("    \\centering\n")
        f.write("    \\resizebox{\\textwidth}{!}{%\n")
        f.write("    \\begin{tabular}{rlcccccccc}\n")
        f.write("    \\toprule\n")
        f.write("    \\multirow{2}{*}{\\textbf{Rank}} & \\multirow{2}{*}{\\textbf{Model}} & ")
        f.write("\\multicolumn{4}{c}{\\textbf{J-LLM Evaluators}} & ")
        f.write("\\multicolumn{3}{c}{\\textbf{Modalities}} \\\\\n")
        f.write("    \\cmidrule(lr){3-6} \\cmidrule(lr){7-9}\n")
        f.write("    & & \\textbf{4.1-mini} & \\textbf{4.1-nano} & \\textbf{4o-mini} & \\textbf{Avg} & ")
        f.write("\\textbf{Ref-Doc} & \\textbf{Ref} & \\textbf{Doc} \\\\\n")
        f.write("    \\midrule\n")
        
        for _, row in df_results.iterrows():
            f.write(f"    {row['Rank']} & {row['Model']} & ")
            f.write(f"{row['J-LLM-4.1-mini']:.2f} & {row['J-LLM-4.1-nano']:.2f} & {row['J-LLM-4o-mini']:.2f} & ")
            f.write(f"{row['Avg-J-LLM']:.2f} & {row['Ref-Doc']:.2f} & {row['Ref']:.2f} & {row['Doc']:.2f} \\\\\n")
        
        f.write("    \\bottomrule\n")
        f.write("    \\end{tabular}\n")
        f.write("    }\n")
        f.write("    \\caption{Comparison of three J-LLM evaluators (GPT-4.1-mini, GPT-4.1-nano, GPT-4o-mini) ")
        f.write("across common base models. The table shows Reference-Document scores from each evaluator ")
        f.write("and the three modality scores (Ref-Doc, Ref, Doc) from GPT-4.1-mini.}\n")
        f.write("    \\label{tab:jllm_comparison}\n")
        f.write("\\end{table*}\n")
    
    print("LaTeX表格已保存到: paper/out/jllm_comparison.tex")
    
    # 打印前10名模型
    print("\n" + "="*80)
    print("Top 10 模型 (按平均J-LLM分数排序)")
    print("="*80)
    print(df_results.head(10).to_string(index=False))
    
    print("\n完成！")

if __name__ == "__main__":
    main()
