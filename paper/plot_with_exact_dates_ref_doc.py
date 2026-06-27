"""
使用精确日期绘制MMLU风格的图表（只使用J-LLM-Ref-Doc分数）
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
    'claude-4-sonnet-20250514': {'company': 'Anthropic', 'size': None, 'date': '2025-05-22'},
    'o3': {'company': 'OpenAI', 'size': None, 'date': '2025-04-16'},
    'claude-3-7-sonnet-20250219': {'company': 'Anthropic', 'size': None, 'date': '2025-02-24'},
    'o4-mini': {'company': 'OpenAI', 'size': None, 'date': '2025-04-16'},
    'qwen3-235b-a22b': {'company': 'Alibaba', 'size': 235, 'date': '2025-07-25'},
    'Gemini-2.5-pro': {'company': 'Google', 'size': None, 'date': '2025-03-25'},
    'gpt-4.1-mini': {'company': 'OpenAI', 'size': None, 'date': '2025-04-14'},
    'gpt-4o-mini': {'company': 'OpenAI', 'size': None, 'date': '2024-07-18'},
    'llama4_maverick': {'company': 'Meta', 'size': 400, 'date': '2025-04-05'},
    'llama4_scout': {'company': 'Meta', 'size': 109, 'date': '2025-04-05'},
    'llama-3.3-70b-instruct': {'company': 'Meta', 'size': 70, 'date': '2024-12-06'},
    'deepseek-r1-32b': {'company': 'DeepSeek', 'size': 32, 'date': '2025-01-20'},
    'gpt-4.1-nano': {'company': 'OpenAI', 'size': None, 'date': '2025-04-14'},
    'llama-3.1-70b-instruct': {'company': 'Meta', 'size': 70, 'date': '2024-07-23'},
    'gpt-4.1': {'company': 'OpenAI', 'size': None, 'date': '2025-04-14'},
    'Gemini-1.5-pro': {'company': 'Google', 'size': None, 'date': '2024-02-15'},
    'codestral-22b-instruct-v0.1': {'company': 'Mistral', 'size': 22, 'date': '2024-05-29'},
    'llama-3.1-405b-instruct': {'company': 'Meta', 'size': 405, 'date': '2024-07-23'},
    'mixtral-8x22b-instruct-v0.1': {'company': 'Mistral', 'size': 176, 'date': '2024-04-17'},
    'llama-3.1-8b-instruct': {'company': 'Meta', 'size': 8, 'date': '2024-07-23'},
    'mistral-nemo-12b-instruct': {'company': 'Mistral', 'size': 12, 'date': '2024-07-18'},
    'deepseek-r1': {'company': 'DeepSeek', 'size': 67, 'date': '2025-01-20'},
    'mistral-large-latest': {'company': 'Mistral', 'size': None, 'date': '2024-02-26'},
    'gemma-2-27b-it': {'company': 'Google', 'size': 27, 'date': '2024-06-26'},
    'mixtral-8x7b-instruct-v0.1': {'company': 'Mistral', 'size': 47, 'date': '2023-12-11'},
    'claude-3-5-sonnet': {'company': 'Anthropic', 'size': None, 'date': '2024-06-21'},
    'deepseek-r1-8b': {'company': 'DeepSeek', 'size': 8, 'date': '2025-01-20'},
    'gpt-4o': {'company': 'OpenAI', 'size': None, 'date': '2024-05-13'},
    'nemotron-4-340b-instruct': {'company': 'NVIDIA', 'size': 340, 'date': '2024-06-14'},
    'gemma-2-9b-it': {'company': 'Google', 'size': 9, 'date': '2024-06-27'},
    'gemma-2-2b-it': {'company': 'Google', 'size': 2, 'date': '2024-07-31'},
    'mamba-codestral-7b-v0.1': {'company': 'Mistral', 'size': 7, 'date': '2024-07-16'},
    'gemma-3-1b-it': {'company': 'Google', 'size': 1, 'date': '2025-03-12'},
    'phi-3-mini-128k-instruct': {'company': 'Microsoft', 'size': 3.8, 'date': '2024-04-24'},
    'phi-3-medium-128k-instruct': {'company': 'Microsoft', 'size': 14, 'date': '2024-05-21'},
}

def extract_data_from_latex():
    """从LaTeX表格中提取数据（只使用J-LLM-Ref-Doc）"""
    with open('paper/out/model_info_table.tex', 'r', encoding='utf-8') as f:
        content = f.read()
    
    lines = content.split('\n')
    data = []
    
    for line in lines:
        if any(keyword in line for keyword in ['claude', 'o3', 'o4', 'gpt', 'Gemini', 'llama', 'deepseek', 'codestral', 'mixtral', 'mistral', 'gemma', 'phi', 'nemotron', 'qwen', 'mamba']):
            parts = [p.strip() for p in line.split('&')]
            if len(parts) >= 12:
                model_display = parts[0].strip()
                jllm_ref_doc = parts[6].strip()  # 只使用J-LLM-Ref-Doc
                exact_date = parts[11].strip()
                
                model_display = re.sub(r'\\[a-zA-Z]+\{?\}?', '', model_display).strip()
                exact_date = re.sub(r'\\\\$', '', exact_date).strip()
                
                try:
                    jllm_score = float(jllm_ref_doc)  # 直接使用J-LLM-Ref-Doc
                    release_date = datetime.strptime(exact_date, '%Y-%m-%d')
                    
                    model_orig = None
                    for key in MODEL_METADATA.keys():
                        key_simple = key.replace('-instruct', '').replace('-v0.1', '').replace('_', '-')
                        if key_simple in model_display or model_display in key_simple:
                            model_orig = key
                            break
                    
                    if model_orig and release_date:
                        metadata = MODEL_METADATA[model_orig]
                        data.append({
                            'model': model_display,
                            'model_orig': model_orig,
                            'release_date': release_date,
                            'j_llm_ref_doc': jllm_score,
                            'company': metadata['company'],
                            'size': metadata['size'] if metadata['size'] else 10
                        })
                except Exception as e:
                    print(f"跳过行: {line[:50]}... 错误: {e}")
                    continue
    
    return pd.DataFrame(data)

def plot_with_exact_dates():
    """使用精确日期绘制MMLU风格的图表（只使用J-LLM-Ref-Doc）"""
    df = extract_data_from_latex()
    
    if len(df) == 0:
        print("未提取到数据")
        return
    
    print(f"成功提取 {len(df)} 个模型的数据")
    print(f"日期范围: {df['release_date'].min()} 到 {df['release_date'].max()}")
    
    # 设置matplotlib参数
    plt.rcParams.update({
        'font.family': 'sans-serif',
        'font.size': 11,
        'axes.labelsize': 14,
        'axes.titlesize': 16,
        'xtick.labelsize': 12,
        'ytick.labelsize': 12,
    })
    
    fig, ax = plt.subplots(figsize=(16, 8))
    
    # 定义公司颜色
    company_colors = {
        'Anthropic': '#FF4444',
        'OpenAI': '#AA66CC',
        'Google': '#FFBB33',
        'Meta': '#00C851',
        'Microsoft': '#33B5E5',
        'Mistral': '#00BCD4',
        'DeepSeek': '#FF4081',
        'Alibaba': '#FFD700',
        'NVIDIA': '#5C6BC0',
    }
    
    # 绘制散点图
    for company in df['company'].unique():
        company_data = df[df['company'] == company]
        
        sizes = []
        for _, row in company_data.iterrows():
            if row['size'] > 10:
                size = 100 + np.log10(row['size']) * 400
            else:
                size = 500
            sizes.append(size)
        
        ax.scatter(company_data['release_date'], company_data['j_llm_ref_doc'],
                  s=sizes, alpha=0.7, c=company_colors.get(company, '#95A5A6'),
                  label=company.lower(), edgecolors='none', linewidths=0, zorder=3)
    
    # 添加模型名称标注
    try:
        from adjustText import adjust_text
        use_adjust = True
    except:
        use_adjust = False
    
    texts = []
    for _, row in df.iterrows():
        text = ax.text(row['release_date'], row['j_llm_ref_doc'], row['model'],
                      fontsize=9, alpha=1.0, color='black',
                      ha='center', va='center', fontweight='normal')
        texts.append(text)
    
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
    dates_numeric = mdates.date2num(df['release_date'])
    slope, intercept, r_value, p_value, std_err = stats.linregress(dates_numeric, df['j_llm_ref_doc'])
    
    x_range = pd.date_range(start=df['release_date'].min(), 
                           end=df['release_date'].max(), 
                           freq='D')
    y_range = slope * mdates.date2num(x_range) + intercept
    ax.plot(x_range, y_range, 'k-', linewidth=2, alpha=0.4, zorder=2)
    
    # 添加基准线
    avg_score = df['j_llm_ref_doc'].mean()
    ax.axhline(y=avg_score, color='gray', linestyle='--', linewidth=1.5, alpha=0.5, zorder=1)
    ax.text(df['release_date'].min(), avg_score + 1, 
           f'{avg_score:.1f} = average', fontsize=11, color='gray', style='italic')
    
    excellent_threshold = 40
    ax.axhline(y=excellent_threshold, color='green', linestyle=':', linewidth=1.5, alpha=0.4, zorder=1)
    ax.text(df['release_date'].min(), excellent_threshold + 1, 
           f'▲ {excellent_threshold}+ EXCELLENT ▲', fontsize=11, color='green', 
           style='italic', fontweight='bold')
    
    # 添加统计信息框（右上角）
    stats_text = f'r = {r_value:.3f}\nR² = {r_value**2:.3f}\np = {p_value:.4f}'
    ax.text(0.98, 0.98, stats_text, transform=ax.transAxes,
           fontsize=11, verticalalignment='top', horizontalalignment='right',
           bbox=dict(boxstyle='round,pad=0.5', facecolor='white', 
                    edgecolor='black', linewidth=1.5, alpha=0.9),
           family='monospace', fontweight='bold')
    
    # 设置坐标轴
    ax.set_xlabel('Release Date', fontsize=15, fontweight='bold', labelpad=10)
    ax.set_ylabel('J-LLM-Ref-Doc Score', fontsize=15, fontweight='bold', labelpad=10)
    ax.set_title('', fontsize=17, fontweight='bold', pad=15)
    
    # 设置x轴格式
    ax.xaxis.set_major_formatter(mdates.DateFormatter('%Y-%m'))
    ax.xaxis.set_major_locator(mdates.MonthLocator(interval=6))
    plt.xticks(rotation=45, ha='right')
    
    # 设置y轴范围
    ax.set_ylim(15, 55)
    
    # 网格
    ax.grid(True, alpha=0.2, linestyle='-', linewidth=0.8, color='#DDDDDD', zorder=0)
    ax.set_axisbelow(True)
    
    # 背景色
    ax.set_facecolor('#FFFFFF')
    
    # 边框
    for spine in ax.spines.values():
        spine.set_linewidth(1)
        spine.set_edgecolor('#E0E0E0')
    
    # 图例
    legend = ax.legend(loc='upper left', frameon=False,
                      shadow=False, ncol=4, fontsize=11,
                      borderpad=0.5, labelspacing=0.3, columnspacing=1,
                      markerscale=0.8)
    
    for handle in legend.legend_handles:
        handle.set_sizes([100])
    
    # 调整布局
    plt.tight_layout()
    
    # 保存图片
    output_path = 'paper/out/jllm_ref_doc_vs_exact_dates.png'
    plt.savefig(output_path, bbox_inches='tight', dpi=300, facecolor='white')
    print(f"图表已保存到: {output_path}")
    
    output_path_pdf = 'paper/out/jllm_ref_doc_vs_exact_dates.pdf'
    plt.savefig(output_path_pdf, bbox_inches='tight', format='pdf', facecolor='white')
    print(f"PDF图表已保存到: {output_path_pdf}")
    
    plt.close()
    
    # 打印统计信息
    print(f"\n数据统计:")
    print(f"总模型数: {len(df)}")
    print(f"J-LLM-Ref-Doc分数范围: [{df['j_llm_ref_doc'].min():.2f}, {df['j_llm_ref_doc'].max():.2f}]")
    print(f"平均分数: {df['j_llm_ref_doc'].mean():.2f}")
    print(f"\n线性回归:")
    print(f"斜率: {slope*365:.4f} (每年变化)")
    print(f"R-squared: {r_value**2:.4f}")
    print(f"p-value: {p_value:.6f}")

if __name__ == "__main__":
    plot_with_exact_dates()
