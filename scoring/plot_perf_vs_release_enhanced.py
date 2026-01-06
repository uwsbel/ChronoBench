"""
绘制性能 vs 发布日期的图表（增强版：按公司分组）
"""
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
from datetime import datetime
import re
import numpy as np
from matplotlib.patches import Rectangle

def parse_release_date(release_str):
    """
    将季度字符串（如2024-Q1）转换为日期
    """
    match = re.match(r'(\d{4})-Q(\d)', release_str)
    if match:
        year = int(match.group(1))
        quarter = int(match.group(2))
        # 季度转换为月份：Q1=1月，Q2=4月，Q3=7月，Q4=10月
        month = (quarter - 1) * 3 + 1
        return datetime(year, month, 15)  # 使用月中日期
    return None

def extract_data_from_latex():
    """
    从LaTeX表格中提取数据
    """
    with open('scoring/out/model_info_table.tex', 'r', encoding='utf-8') as f:
        content = f.read()
    
    lines = content.split('\n')
    data = []
    
    for line in lines:
        if any(keyword in line for keyword in ['claude', 'o3', 'o4', 'gpt', 'Gemini', 'llama', 'deepseek', 'codestral', 'mixtral', 'mistral', 'gemma', 'phi', 'nemotron', 'qwen', 'mamba']):
            # 解析数据行
            parts = [p.strip() for p in line.split('&')]
            if len(parts) >= 11:
                model = parts[0].strip()
                company = parts[1].strip()
                open_weights = parts[2].strip()
                release = parts[4].strip()
                perf = parts[6].strip()  # J-LLM-Ref-Doc
                
                # 移除LaTeX命令
                model = re.sub(r'\\[a-zA-Z]+\{?\}?', '', model).strip()
                company = re.sub(r'\\[a-zA-Z]+\{?\}?', '', company).strip()
                open_weights = 'Yes' if 'checkmark' in open_weights else 'No'
                
                try:
                    perf_value = float(perf)
                    release_date = parse_release_date(release)
                    if release_date:
                        data.append({
                            'model': model,
                            'company': company,
                            'open_weights': open_weights,
                            'release': release,
                            'release_date': release_date,
                            'performance': perf_value
                        })
                except:
                    continue
    
    return pd.DataFrame(data)

def plot_performance_vs_release():
    """
    绘制性能 vs 发布日期的图表（按公司分组）
    """
    df = extract_data_from_latex()
    
    if len(df) == 0:
        print("未提取到数据")
        return
    
    # 设置图表样式
    try:
        plt.style.use('seaborn-v0_8-whitegrid')
    except:
        plt.style.use('seaborn-whitegrid')
    
    plt.rcParams['figure.dpi'] = 150
    plt.rcParams['savefig.dpi'] = 300
    plt.rcParams['font.size'] = 10
    
    fig, ax = plt.subplots(figsize=(14, 8))
    
    # 定义公司颜色映射
    companies = df['company'].unique()
    colors = plt.cm.tab20(np.linspace(0, 1, len(companies)))
    company_colors = dict(zip(companies, colors))
    
    # 定义标记样式（按开源状态）
    markers = {'Yes': 'o', 'No': 's'}  # 开源用圆圈，闭源用方块
    
    # 绘制散点图
    for company in companies:
        company_data = df[df['company'] == company]
        for open_status in ['Yes', 'No']:
            data_subset = company_data[company_data['open_weights'] == open_status]
            if len(data_subset) > 0:
                marker = markers[open_status]
                label = f"{company} ({'Open' if open_status == 'Yes' else 'Closed'})"
                ax.scatter(data_subset['release_date'], data_subset['performance'], 
                          s=120, alpha=0.7, c=[company_colors[company]], 
                          marker=marker, label=label if len(company_data) <= 2 else None,
                          edgecolors='black', linewidths=0.8, zorder=3)
    
    # 添加趋势线
    dates_num = mdates.date2num(df['release_date'])
    z = np.polyfit(dates_num, df['performance'], 1)
    p = np.poly1d(z)
    
    x_trend = pd.date_range(start=df['release_date'].min(), 
                           end=df['release_date'].max(), 
                           freq='ME')
    ax.plot(x_trend, p(mdates.date2num(x_trend)), 
           "r--", alpha=0.6, linewidth=2.5, 
           label=f'Linear Trend (slope={z[0]*365:.2f} per year)', zorder=2)
    
    # 标注top 3模型
    top_models = df.nlargest(3, 'performance')
    for _, row in top_models.iterrows():
        ax.annotate(row['model'], 
                   xy=(row['release_date'], row['performance']),
                   xytext=(8, 8), textcoords='offset points',
                   fontsize=9, alpha=0.8, fontweight='bold',
                   bbox=dict(boxstyle='round,pad=0.4', facecolor='yellow', alpha=0.6, edgecolor='black'),
                   arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=0.2', alpha=0.6))
    
    # 设置标签和标题
    ax.set_xlabel('Release Date', fontsize=13, fontweight='bold')
    ax.set_ylabel('J-LLM-Ref-Doc Performance Score', fontsize=13, fontweight='bold')
    ax.set_title('Model Performance vs Release Date', fontsize=15, fontweight='bold', pad=20)
    
    # 格式化x轴日期
    ax.xaxis.set_major_formatter(mdates.DateFormatter('%Y-%m'))
    ax.xaxis.set_major_locator(mdates.MonthLocator(interval=3))
    plt.xticks(rotation=45, ha='right')
    
    # 设置y轴范围
    ax.set_ylim(df['performance'].min() - 2, df['performance'].max() + 3)
    
    # 添加网格
    ax.grid(True, alpha=0.3, linestyle='--', zorder=1)
    
    # 添加图例（简化版）
    # 只显示主要公司
    from matplotlib.lines import Line2D
    legend_elements = [
        Line2D([0], [0], marker='o', color='w', markerfacecolor='gray', 
               markersize=10, label='Open Weights', markeredgecolor='black', markeredgewidth=0.8),
        Line2D([0], [0], marker='s', color='w', markerfacecolor='gray', 
               markersize=10, label='Closed Source', markeredgecolor='black', markeredgewidth=0.8),
        Line2D([0], [0], color='red', linestyle='--', linewidth=2.5, label='Linear Trend')
    ]
    ax.legend(handles=legend_elements, loc='upper left', fontsize=10, framealpha=0.9)
    
    # 添加公司图例（右侧）
    company_legend_elements = [Line2D([0], [0], marker='o', color='w', 
                                      markerfacecolor=company_colors[comp], 
                                      markersize=12, label=comp, 
                                      markeredgecolor='black', markeredgewidth=0.8)
                               for comp in sorted(companies)]
    ax2 = ax.twinx()
    ax2.set_yticks([])
    ax2.legend(handles=company_legend_elements, loc='upper right', 
              fontsize=9, framealpha=0.9, title='Companies', title_fontsize=10)
    
    # 调整布局
    plt.tight_layout()
    
    # 保存图片
    output_path = 'scoring/out/performance_vs_release_date.png'
    plt.savefig(output_path, bbox_inches='tight', dpi=300)
    print(f"图表已保存到: {output_path}")
    
    # 也保存为PDF格式（适合论文）
    output_path_pdf = 'scoring/out/performance_vs_release_date.pdf'
    plt.savefig(output_path_pdf, bbox_inches='tight', format='pdf')
    print(f"PDF图表已保存到: {output_path_pdf}")
    
    plt.close()
    
    # 打印统计信息
    print(f"\n数据统计:")
    print(f"总模型数: {len(df)}")
    print(f"开源模型: {len(df[df['open_weights'] == 'Yes'])}")
    print(f"闭源模型: {len(df[df['open_weights'] == 'No'])}")
    print(f"性能范围: {df['performance'].min():.1f} - {df['performance'].max():.1f}")
    print(f"平均性能: {df['performance'].mean():.2f}")
    print(f"\n按公司统计:")
    for company in sorted(df['company'].unique()):
        comp_data = df[df['company'] == company]
        print(f"  {company}: {len(comp_data)} models, avg perf: {comp_data['performance'].mean():.2f}")

if __name__ == "__main__":
    plot_performance_vs_release()
