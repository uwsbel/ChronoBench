"""
绘制类似MMLU风格的图表：J-LLM分数 vs 发布时间
"""
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
from datetime import datetime
import re
import numpy as np
from scipy import stats

# 模型元数据
MODEL_METADATA = {
    'claude-4-sonnet-20250514': {'company': 'Anthropic', 'size': None, 'release': '2025-Q2'},
    'o3': {'company': 'OpenAI', 'size': None, 'release': '2025-Q2'},
    'claude-3-7-sonnet-20250219': {'company': 'Anthropic', 'size': None, 'release': '2025-Q1'},
    'o4-mini': {'company': 'OpenAI', 'size': None, 'release': '2025-Q2'},
    'qwen3-235b-a22b': {'company': 'Alibaba', 'size': 235, 'release': '2025-Q2'},
    'Gemini-2.5-pro': {'company': 'Google', 'size': None, 'release': '2025-Q2'},
    'gpt-4.1-mini': {'company': 'OpenAI', 'size': None, 'release': '2025-Q2'},
    'gpt-4o-mini': {'company': 'OpenAI', 'size': None, 'release': '2024-Q3'},
    'llama4_maverick': {'company': 'Meta', 'size': 400, 'release': '2025-Q2'},
    'llama4_scout': {'company': 'Meta', 'size': 109, 'release': '2025-Q2'},
    'llama-3.3-70b-instruct': {'company': 'Meta', 'size': 70, 'release': '2024-Q4'},
    'deepseek-r1-32b': {'company': 'DeepSeek', 'size': 32, 'release': '2025-Q1'},
    'gpt-4.1-nano': {'company': 'OpenAI', 'size': None, 'release': '2025-Q2'},
    'llama-3.1-70b-instruct': {'company': 'Meta', 'size': 70, 'release': '2024-Q3'},
    'gpt-4.1': {'company': 'OpenAI', 'size': None, 'release': '2025-Q2'},
    'Gemini-1.5-pro': {'company': 'Google', 'size': None, 'release': '2024-Q1'},
    'codestral-22b-instruct-v0.1': {'company': 'Mistral', 'size': 22, 'release': '2024-Q2'},
    'llama-3.1-405b-instruct': {'company': 'Meta', 'size': 405, 'release': '2024-Q3'},
    'mixtral-8x22b-instruct-v0.1': {'company': 'Mistral', 'size': 176, 'release': '2024-Q2'},
    'llama-3.1-8b-instruct': {'company': 'Meta', 'size': 8, 'release': '2024-Q3'},
    'mistral-nemo-12b-instruct': {'company': 'Mistral', 'size': 12, 'release': '2024-Q3'},
    'deepseek-r1': {'company': 'DeepSeek', 'size': 67, 'release': '2024-Q4'},
    'mistral-large-latest': {'company': 'Mistral', 'size': None, 'release': '2024-Q1'},
    'gemma-2-27b-it': {'company': 'Google', 'size': 27, 'release': '2024-Q2'},
    'mixtral-8x7b-instruct-v0.1': {'company': 'Mistral', 'size': 47, 'release': '2023-Q4'},
    'claude-3-5-sonnet': {'company': 'Anthropic', 'size': None, 'release': '2024-Q2'},
    'deepseek-r1-8b': {'company': 'DeepSeek', 'size': 8, 'release': '2025-Q1'},
    'gpt-4o': {'company': 'OpenAI', 'size': None, 'release': '2024-Q2'},
    'nemotron-4-340b-instruct': {'company': 'NVIDIA', 'size': 340, 'release': '2024-Q2'},
    'gemma-2-9b-it': {'company': 'Google', 'size': 9, 'release': '2024-Q2'},
    'gemma-2-2b-it': {'company': 'Google', 'size': 2, 'release': '2024-Q2'},
    'mamba-codestral-7b-v0.1': {'company': 'Mistral', 'size': 7, 'release': '2024-Q3'},
    'gemma-3-1b-it': {'company': 'Google', 'size': 1, 'release': '2025-Q1'},
    'phi-3-mini-128k-instruct': {'company': 'Microsoft', 'size': 3.8, 'release': '2024-Q2'},
    'phi-3-medium-128k-instruct': {'company': 'Microsoft', 'size': 14, 'release': '2024-Q2'},
}

def parse_release_date(release_str):
    """将季度字符串转换为日期（小数年份）"""
    match = re.match(r'(\d{4})-Q(\d)', release_str)
    if match:
        year = int(match.group(1))
        quarter = int(match.group(2))
        # 转换为小数年份：Q1=0.125, Q2=0.375, Q3=0.625, Q4=0.875
        decimal_year = year + (quarter - 0.5) / 4.0
        return decimal_year
    return None

def extract_data_from_latex():
    """从LaTeX表格中提取数据"""
    with open('scoring/out/model_info_table.tex', 'r', encoding='utf-8') as f:
        content = f.read()
    
    lines = content.split('\n')
    data = []
    
    for line in lines:
        if any(keyword in line for keyword in ['claude', 'o3', 'o4', 'gpt', 'Gemini', 'llama', 'deepseek', 'codestral', 'mixtral', 'mistral', 'gemma', 'phi', 'nemotron', 'qwen', 'mamba']):
            parts = [p.strip() for p in line.split('&')]
            if len(parts) >= 11:
                model_display = parts[0].strip()
                release = parts[4].strip()
                jllm_ref_doc = parts[6].strip()
                jllm_ref = parts[7].strip()
                jllm_doc = parts[8].strip()
                
                model_display = re.sub(r'\\[a-zA-Z]+\{?\}?', '', model_display).strip()
                release = re.sub(r'\\[a-zA-Z]+\{?\}?', '', release).strip()
                jllm_ref_doc = re.sub(r'\\\\$', '', jllm_ref_doc).strip()
                
                try:
                    # 计算平均J-LLM分数
                    jllm_avg = (float(jllm_ref_doc) + float(jllm_ref) + float(jllm_doc)) / 3.0
                    release_decimal = parse_release_date(release)
                    
                    # 匹配原始模型名
                    model_orig = None
                    for key in MODEL_METADATA.keys():
                        key_simple = key.replace('-instruct', '').replace('-v0.1', '').replace('_', '-')
                        if key_simple in model_display or model_display in key_simple:
                            model_orig = key
                            break
                    
                    if model_orig and release_decimal:
                        metadata = MODEL_METADATA[model_orig]
                        data.append({
                            'model': model_display,
                            'model_orig': model_orig,
                            'release_year': release_decimal,
                            'j_llm': jllm_avg,
                            'company': metadata['company'],
                            'size': metadata['size'] if metadata['size'] else 10  # 默认大小
                        })
                except:
                    continue
    
    return pd.DataFrame(data)

def plot_mmlu_style():
    """绘制MMLU风格的图表"""
    df = extract_data_from_latex()
    
    if len(df) == 0:
        print("未提取到数据")
        return
    
    # 设置matplotlib参数
    plt.rcParams.update({
        'font.family': 'sans-serif',
        'font.size': 9,
        'axes.labelsize': 11,
        'axes.titlesize': 12,
        'xtick.labelsize': 10,
        'ytick.labelsize': 10,
    })
    
    # 创建图表
    fig, ax = plt.subplots(figsize=(16, 8))
    
    # 定义公司颜色（更鲜艳的配色）
    company_colors = {
        'Anthropic': '#FF4444',      # 亮红色
        'OpenAI': '#AA66CC',         # 亮紫色
        'Google': '#FFBB33',         # 亮橙色
        'Meta': '#00C851',           # 亮绿色
        'Microsoft': '#33B5E5',      # 亮蓝色
        'Mistral': '#00BCD4',        # 亮青色
        'DeepSeek': '#FF4081',       # 亮粉色
        'Alibaba': '#FFD700',        # 金色
        'NVIDIA': '#5C6BC0',         # 亮靛蓝
    }
    
    # 绘制散点图
    for company in df['company'].unique():
        company_data = df[df['company'] == company]
        
        # 点的大小根据模型大小（开源模型）或固定大小（闭源模型）
        sizes = []
        for _, row in company_data.iterrows():
            if row['size'] > 10:  # 有实际大小数据
                # 对数缩放
                size = 100 + np.log10(row['size']) * 400
            else:
                size = 500  # 闭源模型使用固定较大尺寸
            sizes.append(size)
        
        ax.scatter(company_data['release_year'], company_data['j_llm'],
                  s=sizes, alpha=0.7, c=company_colors.get(company, '#95A5A6'),
                  label=company.lower(), edgecolors='none', linewidths=0, zorder=3)
    
    # 添加模型名称标注（所有模型）
    try:
        from adjustText import adjust_text
        use_adjust = True
    except:
        use_adjust = False
    
    texts = []
    
    # 先添加所有标注（纯文字，无背景）
    for _, row in df.iterrows():
        text = ax.text(row['release_year'], row['j_llm'], row['model'],
                      fontsize=7.5, alpha=1.0, color='black',
                      ha='center', va='center', fontweight='normal')
        texts.append(text)
    
    # 使用adjustText避免标签重叠
    if use_adjust:
        try:
            adjust_text(texts, 
                       arrowprops=dict(arrowstyle='-', color='gray', lw=0.5, alpha=0.6),
                       expand_points=(1.5, 1.5), 
                       expand_text=(1.2, 1.2),
                       force_points=(0.5, 0.5),
                       force_text=(0.5, 0.5),
                       ax=ax)
        except:
            pass
    
    # 添加线性回归线
    slope, intercept, r_value, p_value, std_err = stats.linregress(df['release_year'], df['j_llm'])
    x_range = np.array([df['release_year'].min() - 0.3, df['release_year'].max() + 0.3])
    y_range = slope * x_range + intercept
    ax.plot(x_range, y_range, 'k-', linewidth=2, alpha=0.4, zorder=2)
    
    # 添加基准线
    # 平均分数线
    avg_score = df['j_llm'].mean()
    ax.axhline(y=avg_score, color='gray', linestyle='--', linewidth=1.5, alpha=0.5, zorder=1)
    ax.text(df['release_year'].min(), avg_score + 1, 
           f'{avg_score:.1f} = average', fontsize=9, color='gray', style='italic')
    
    # 优秀模型基准线（假设40+为优秀）
    excellent_threshold = 40
    ax.axhline(y=excellent_threshold, color='green', linestyle=':', linewidth=1.5, alpha=0.4, zorder=1)
    ax.text(df['release_year'].min(), excellent_threshold + 1, 
           f'▲ {excellent_threshold}+ EXCELLENT ▲', fontsize=9, color='green', 
           style='italic', fontweight='bold')
    
    # 设置坐标轴
    ax.set_xlabel('Release Year', fontsize=12, fontweight='bold', labelpad=10)
    ax.set_ylabel('J-LLM Score', fontsize=12, fontweight='bold', labelpad=10)
    ax.set_title('', fontsize=14, fontweight='bold', pad=15)
    
    # 设置x轴范围和刻度
    ax.set_xlim(2023.5, 2025.7)
    ax.set_xticks([2023.5, 2024, 2024.5, 2025, 2025.5])
    ax.set_xticklabels(['H2 2023', '2024', 'H2 2024', '2025', 'H2 2025'])
    
    # 设置y轴范围
    ax.set_ylim(15, 55)
    
    # 网格（更淡，更接近参考图）
    ax.grid(True, alpha=0.15, linestyle='-', linewidth=0.5, color='#DDDDDD', zorder=0)
    ax.set_axisbelow(True)
    
    # 背景色（纯白，更接近参考图）
    ax.set_facecolor('#FFFFFF')
    
    # 边框（更淡）
    for spine in ax.spines.values():
        spine.set_linewidth(1)
        spine.set_edgecolor('#E0E0E0')
    
    # 图例 - 无边框
    legend = ax.legend(loc='upper left', frameon=False,  # frameon=False 去掉边框
                      shadow=False, ncol=4, 
                      borderpad=0.5, labelspacing=0.3, columnspacing=1,
                      markerscale=0.8)  # 控制图例中标记的大小
    
    # 强制统一图例中所有点的大小
    for handle in legend.legend_handles:
        handle.set_sizes([100])  # 统一大小
    
    # 调整布局
    plt.tight_layout()
    
    # 保存图片
    output_path = 'scoring/out/jllm_vs_release_mmlu_style.png'
    plt.savefig(output_path, bbox_inches='tight', dpi=300, facecolor='white')
    print(f"图表已保存到: {output_path}")
    
    output_path_pdf = 'scoring/out/jllm_vs_release_mmlu_style.pdf'
    plt.savefig(output_path_pdf, bbox_inches='tight', format='pdf', facecolor='white')
    print(f"PDF图表已保存到: {output_path_pdf}")
    
    plt.close()
    
    # 打印统计信息
    print(f"\n数据统计:")
    print(f"总模型数: {len(df)}")
    print(f"J-LLM分数范围: [{df['j_llm'].min():.2f}, {df['j_llm'].max():.2f}]")
    print(f"平均分数: {df['j_llm'].mean():.2f}")
    print(f"\n线性回归:")
    print(f"斜率: {slope:.4f} (每年变化)")
    print(f"R-squared: {r_value**2:.4f}")
    print(f"p-value: {p_value:.4f}")

if __name__ == "__main__":
    plot_mmlu_style()
