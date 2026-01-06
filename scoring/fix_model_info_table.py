"""
修复并更新模型信息表格，正确添加J-LLM-Ref和J-LLM-Doc两列
"""
import pandas as pd
import re

def calculate_model_metrics():
    """
    计算每个模型的score_reference和score_document的平均值
    """
    df = pd.read_csv('scoring/out/all_metrics_merged_pretrain_only_advanced.csv')
    
    # 按模型分组，计算平均值
    model_metrics = df.groupby('model').agg({
        'score_reference': 'mean',
        'score_document': 'mean'
    }).reset_index()
    
    # 转换为整数
    model_metrics['score_reference'] = model_metrics['score_reference'].round().astype(int)
    model_metrics['score_document'] = model_metrics['score_document'].round().astype(int)
    
    # 创建模型名称映射字典
    model_map = {}
    for _, row in model_metrics.iterrows():
        model_full = row['model']
        # 创建多个可能的简化名称
        simplified = model_full.replace('_', '-').replace('-instruct', '').replace('-v0.1', '').replace('-instruct-v0.1', '-v0.1')
        model_map[model_full.lower()] = row
        model_map[simplified.lower()] = row
        # 也添加不带版本号的版本
        simplified_no_version = simplified.replace('-v0.1', '').replace('-20250514', '').replace('-20250219', '').replace('-20250219', '')
        if simplified_no_version.lower() != simplified.lower():
            model_map[simplified_no_version.lower()] = row
    
    return model_map

def fix_latex_table():
    """
    修复并更新LaTeX表格
    """
    model_map = calculate_model_metrics()
    
    # 读取现有的LaTeX表格
    with open('scoring/out/model_info_table.tex', 'r', encoding='utf-8') as f:
        latex_content = f.read()
    
    lines = latex_content.split('\n')
    new_lines = []
    
    for line in lines:
        if '\\begin{tabular}' in line:
            new_lines.append("    \\begin{tabular}{lccccc ccc}")
        elif '\\textbf{Model}' in line and 'J-LLM-Ref' in line:
            # 表头已存在，直接使用
            new_lines.append("    \\textbf{Model} & \\textbf{Company} & \\textbf{Open Weights} & \\textbf{Size} & \\textbf{Release} & \\textbf{Reasoning} & \\textbf{J-LLM-Ref-Doc} & \\textbf{J-LLM-Ref} & \\textbf{J-LLM-Doc} \\\\")
        elif any(keyword in line for keyword in ['claude', 'o3', 'o4', 'gpt', 'Gemini', 'llama', 'deepseek', 'codestral', 'mixtral', 'mistral', 'gemma', 'phi', 'nemotron', 'qwen', 'mamba']):
            # 数据行 - 清理并重新构建
            parts = [p.strip() for p in line.split('&')]
            
            # 移除重复的数据（如果有）
            if len(parts) > 9:
                # 只保留前7列，然后添加新的2列
                parts = parts[:7]
            
            if len(parts) >= 7:
                model_name = parts[0].strip()
                # 移除LaTeX命令
                model_name_clean = re.sub(r'\\[a-zA-Z]+\{?\}?', '', model_name).strip().lower()
                
                # 查找对应的指标
                ref_score = None
                doc_score = None
                
                # 尝试匹配
                for key in model_map:
                    if model_name_clean == key or model_name_clean in key or key in model_name_clean:
                        row = model_map[key]
                        ref_score = row['score_reference']
                        doc_score = row['score_document']
                        break
                
                # 构建新行
                if ref_score is not None and doc_score is not None:
                    new_line = ' & '.join(parts[:7]) + f" & {ref_score} & {doc_score} \\\\"
                    new_lines.append(new_line)
                else:
                    # 如果找不到，添加占位符
                    print(f"警告：未找到模型 {model_name_clean} 的指标数据")
                    new_line = ' & '.join(parts[:7]) + " & -- & -- \\\\"
                    new_lines.append(new_line)
            else:
                new_lines.append(line)
        elif '\\caption' in line:
            new_lines.append("    \\caption{Overview of pretrained S-LLMs evaluated in this study. ``Open Weights'' indicates whether model weights are publicly available. ``Reasoning'' indicates models explicitly positioned or configured for deliberate reasoning by their providers. ``J-LLM-Ref-Doc'', ``J-LLM-Ref'', and ``J-LLM-Doc'' report the average scores across all system categories. Models are ordered by ``J-LLM-Ref-Doc''.}")
        else:
            new_lines.append(line)
    
    return '\n'.join(new_lines)

if __name__ == "__main__":
    updated_latex = fix_latex_table()
    
    # 保存更新后的表格
    with open('scoring/out/model_info_table.tex', 'w', encoding='utf-8') as f:
        f.write(updated_latex)
    
    print("表格已修复并更新！")
    print("\n更新后的表格预览（前20行）:")
    print('\n'.join(updated_latex.split('\n')[:20]))
