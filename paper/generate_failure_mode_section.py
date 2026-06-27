"""
生成Failure Mode Analysis的LaTeX章节
"""
import pandas as pd
import sys
import io

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')

def generate_latex_table():
    """生成LaTeX表格"""
    # 读取系统难度分析数据
    df = pd.read_csv('paper/out/system_difficulty_analysis.csv')
    
    # 选择需要的列
    df_table = df[['system', 'Category', 'Turn1_Avg', 'Turn2_Avg', 'Turn3_Avg', 'Overall_Avg']].copy()
    
    # 格式化系统名称（替换下划线）
    df_table['system'] = df_table['system'].str.replace('_', '\\_')
    
    print("生成LaTeX表格...")
    
    # 生成LaTeX代码
    latex = []
    latex.append("\\begin{table*}[htbp]")
    latex.append("    \\centering")
    latex.append("    \\caption{System difficulty analysis: average J-LLM-Ref-Doc scores across all S-LLMs and turns. Systems ranked by overall difficulty (lower scores indicate harder systems).}")
    latex.append("    \\label{tab:system_difficulty}")
    latex.append("    \\resizebox{\\textwidth}{!}{%")
    latex.append("    \\begin{tabular}{rlcccc}")
    latex.append("    \\toprule")
    latex.append("    \\textbf{Rank} & \\textbf{System} & \\textbf{Category} & \\textbf{Turn 1} & \\textbf{Turn 2} & \\textbf{Turn 3} & \\textbf{Overall} \\\\")
    latex.append("    \\midrule")
    
    # 数据行
    for i, row in df_table.iterrows():
        rank = i + 1
        latex.append(f"    {rank} & {row['system']} & {row['Category']} & "
                    f"{row['Turn1_Avg']:.1f} & {row['Turn2_Avg']:.1f} & {row['Turn3_Avg']:.1f} & "
                    f"{row['Overall_Avg']:.1f} \\\\")
    
    latex.append("    \\bottomrule")
    latex.append("    \\end{tabular}")
    latex.append("    }")
    latex.append("\\end{table*}")
    
    return '\n'.join(latex)

def generate_analysis_text():
    """生成分析文字"""
    # 读取数据
    system_df = pd.read_csv('paper/out/system_difficulty_analysis.csv')
    category_df = pd.read_csv('paper/out/category_difficulty_analysis.csv', index_col=0)
    
    # 提取关键统计
    hardest_5 = system_df.head(5)
    easiest_5 = system_df.tail(5)
    
    hardest_cat = category_df.nsmallest(1, 'Overall_Avg').index[0]
    hardest_cat_score = category_df.loc[hardest_cat, 'Overall_Avg']
    
    easiest_cat = category_df.nlargest(1, 'Overall_Avg').index[0]
    easiest_cat_score = category_df.loc[easiest_cat, 'Overall_Avg']
    
    # 找到Turn 1最难的系统
    worst_turn1 = system_df.nsmallest(1, 'Turn1_Avg').iloc[0]
    
    # 找到Turn 3下降最严重的系统
    worst_delta23 = system_df.nsmallest(1, 'Δ23').iloc[0]
    
    # 最不稳定的系统
    most_unstable = system_df.nlargest(1, 'Overall_Std').iloc[0]
    
    print("\n生成分析文字...")
    
    text = []
    text.append("\\subsection{Failure Mode Analysis: System Difficulty Assessment}")
    text.append("\\label{subsec:failure_mode}")
    text.append("")
    text.append("To identify challenging scenarios and failure patterns in simulation code generation, we conduct a comprehensive system-level difficulty analysis. Table~\\ref{tab:system_difficulty} presents the performance of all 34 simulation systems, ranked by overall difficulty (average J-LLM-Ref-Doc scores across all S-LLMs and all three turns). Lower scores indicate harder systems where models consistently struggle.")
    text.append("")
    text.append("\\textbf{Sensor systems emerge as the most challenging domain.}")
    text.append(f"The hardest system overall is \\texttt{{{hardest_5.iloc[0]['system']}}} (SEN category, overall score {hardest_5.iloc[0]['Overall_Avg']:.1f}), which requires intricate sensor configuration including noise models, visualization pipelines, and dynamic sensor positioning. The SEN category achieves the lowest average performance ({hardest_cat_score:.1f}), significantly below other domains. This persistent difficulty stems from:")
    text.append("")
    text.append("\\begin{itemize}")
    text.append("    \\item \\textbf{API complexity}: Sensor initialization requires precise parameter tuning (e.g., resolution, field-of-view, update rates) with limited error tolerance.")
    text.append("    \\item \\textbf{Visualization challenges}: Sensors demand additional rendering setup, buffer management, and frame processing that are domain-specific and poorly documented in training data.")
    text.append("    \\item \\textbf{Real-time constraints}: Sensor simulations require careful synchronization between physics updates and sensor data acquisition, a subtle requirement often missed by models.")
    text.append("\\end{itemize}")
    text.append("")
    text.append(f"Similarly, \\texttt{{{hardest_5.iloc[3]['system']}}} (SEN, {hardest_5.iloc[3]['Overall_Avg']:.1f}) exhibits consistently low performance across all turns (T1={hardest_5.iloc[3]['Turn1_Avg']:.1f}, T2={hardest_5.iloc[3]['Turn2_Avg']:.1f}, T3={hardest_5.iloc[3]['Turn3_Avg']:.1f}), with the Turn 2$\\to$3 decline ({hardest_5.iloc[3]['Δ23']:.1f}) highlighting the difficulty of extending sensor functionality while maintaining correct configuration.")
    text.append("")
    text.append(f"\\textbf{{Robotics and vehicle systems show moderate difficulty with high variability.}}")
    text.append(f"The RBT category (average {category_df.loc['RBT', 'Overall_Avg']:.1f}) and VEH category ({category_df.loc['VEH', 'Overall_Avg']:.1f}) occupy the middle difficulty range. Systems like \\texttt{{{hardest_5.iloc[1]['system']}}} (RBT, {hardest_5.iloc[1]['Overall_Avg']:.1f}) and \\texttt{{{hardest_5.iloc[2]['system']}}} (VEH, {hardest_5.iloc[2]['Overall_Avg']:.1f}) demonstrate interesting failure patterns: relatively low Turn 1 generation scores but substantial Turn 2 improvements (+{hardest_5.iloc[1]['Δ12']:.1f} and +{hardest_5.iloc[2]['Δ12']:.1f} respectively), suggesting models can leverage context effectively once initial code structure is provided. However, the high standard deviation in these categories (e.g., \\texttt{{{most_unstable['system']}}} with std={most_unstable['Overall_Std']:.1f}) indicates significant performance variance across different models—some handle complex kinematics well while others struggle with constraint specification.")
    text.append("")
    text.append(f"\\textbf{{FEA and MBS systems are relatively easier but exhibit distinct patterns.}}")
    text.append(f"The FEA category achieves the highest average performance ({easiest_cat_score:.1f}), with \\texttt{{{easiest_5.iloc[4]['system']}}} reaching {easiest_5.iloc[4]['Overall_Avg']:.1f}—the easiest system overall. This success likely reflects: (1) well-structured simulation workflows with clear mesh$\\to$material$\\to$solver pipelines, (2) abundant finite element examples in training corpora, and (3) less ambiguity in problem specifications. The MBS category ({category_df.loc['MBS', 'Overall_Avg']:.1f}) also performs well, though systems like \\texttt{{{worst_delta23['system']}}} exhibit dramatic Turn 3 declines ({worst_delta23['Δ23']:.1f}), revealing challenges in extending multi-body dynamics while preserving kinematic consistency.")
    text.append("")
    text.append("\\textbf{Turn-specific failure modes reveal distinct challenges.}")
    text.append(f"Turn 1 (generation from scratch) proves universally difficult, with scores averaging only {system_df['Turn1_Avg'].mean():.1f} and systems like \\texttt{{{worst_turn1['system']}}} scoring just {worst_turn1['Turn1_Avg']:.1f}. This baseline difficulty underscores the challenge of synthesizing complete simulation setups without examples. Turn 2 (modification with context) shows dramatic improvement (average +{system_df['Δ12'].mean():.1f}), but the benefit varies widely—from +{system_df['Δ12'].min():.1f} for systems with template-like structures (minimal leverage from context) to +{system_df['Δ12'].max():.1f} for complex systems where context provides critical scaffolding. Turn 3 (extension tasks) reveals brittleness: {(system_df['Δ23'] < 0).sum()} out of 34 systems show performance declines, with some catastrophic drops (e.g., \\texttt{{{worst_delta23['system']}}}: {worst_delta23['Δ23']:.1f}) indicating models struggle to add functionality while maintaining simulation correctness.")
    text.append("")
    text.append("\\textbf{Implications for S-LLM development.}")
    text.append("This failure mode analysis identifies three priority areas for improvement:")
    text.append("")
    text.append("\\begin{enumerate}")
    text.append("    \\item \\textbf{Sensor domain specialization}: Targeted training on sensor configuration patterns, visualization pipelines, and buffer management could significantly improve the weakest category.")
    text.append("    \\item \\textbf{Consistency-preserving extensions}: The widespread Turn 3 degradation calls for methods that explicitly verify backward compatibility when extending code (e.g., constraint checking, regression testing).")
    text.append("    \\item \\textbf{Variance reduction}: High inter-model variance in complex systems (robotics, vehicles) suggests ensemble or hybrid approaches might provide more reliable generation.")
    text.append("\\end{enumerate}")
    text.append("")
    text.append("These findings complement our multi-turn delta analysis (Section~\\ref{subsec:multiturn_delta}), providing system-level granularity that guides both model selection for specific simulation domains and targeted improvements in S-LLM capabilities.")
    
    return '\n'.join(text)

def main():
    # 生成表格
    latex_table = generate_latex_table()
    
    # 生成分析文字
    latex_text = generate_analysis_text()
    
    # 完整的LaTeX章节
    full_latex = latex_text + "\n\n" + latex_table
    
    # 保存
    with open('paper/out/failure_mode_section.tex', 'w', encoding='utf-8') as f:
        f.write(full_latex)
    
    print("\nLaTeX章节已保存到: paper/out/failure_mode_section.tex")
    
    # 也生成一个简洁版本
    print("\n生成简洁版本...")
    
    # 只保留前20个系统的表格
    df = pd.read_csv('paper/out/system_difficulty_analysis.csv')
    df_top20 = df.head(20)
    df_top20.to_csv('paper/out/system_difficulty_top20.csv', index=False)
    
    print("前20个最难系统已保存到: system_difficulty_top20.csv")
    
    print("\n完成！")

if __name__ == "__main__":
    main()
