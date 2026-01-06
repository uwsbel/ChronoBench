"""
绘制性能 vs 发布日期的图表
"""
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
from datetime import datetime
import re
import numpy as np

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
    绘制性能 vs 发布日期的图表
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
    plt.rcParams['font.family'] = 'DejaVu Sans'
    
    fig, ax = plt.subplots(figsize=(12, 8))
    
    # 按开源状态分组
    open_source = df[df['open_weights'] == 'Yes']
    closed_source = df[df['open_weights'] == 'No']
    
    # 绘制散点图
    scatter1 = ax.scatter(closed_source['release_date'], closed_source['performance'], 
                         s=100, alpha=0.7, c='#1f77b4', label='Closed Source', 
                         edgecolors='black', linewidths=0.5, zorder=3)
    scatter2 = ax.scatter(open_source['release_date'], open_source['performance'], 
                         s=100, alpha=0.7, c='#ff7f0e', label='Open Weights', 
                         edgecolors='black', linewidths=0.5, zorder=3)
    
    # 添加趋势线
    # 将所有日期转换为数值
    dates_num = mdates.date2num(df['release_date'])
    z = np.polyfit(dates_num, df['performance'], 1)
    p = np.poly1d(z)
    
    # 绘制趋势线
    x_trend = pd.date_range(start=df['release_date'].min(), 
                           end=df['release_date'].max(), 
                           freq='ME')  # ME = Month End
    ax.plot(x_trend, p(mdates.date2num(x_trend)), 
           "r--", alpha=0.5, linewidth=2, label=f'Trend (slope={z[0]:.3f})', zorder=2)
    
    # 标注一些重要模型
    top_models = df.nlargest(5, 'performance')
    for _, row in top_models.iterrows():
        ax.annotate(row['model'], 
                   xy=(row['release_date'], row['performance']),
                   xytext=(5, 5), textcoords='offset points',
                   fontsize=8, alpha=0.7,
                   bbox=dict(boxstyle='round,pad=0.3', facecolor='yellow', alpha=0.3))
    
    # 设置标签和标题
    ax.set_xlabel('Release Date', fontsize=12, fontweight='bold')
    ax.set_ylabel('J-LLM-Ref-Doc Performance', fontsize=12, fontweight='bold')
    ax.set_title('Model Performance vs Release Date', fontsize=14, fontweight='bold', pad=20)
    
    # 格式化x轴日期
    ax.xaxis.set_major_formatter(mdates.DateFormatter('%Y-%m'))
    ax.xaxis.set_major_locator(mdates.MonthLocator(interval=3))
    plt.xticks(rotation=45, ha='right')
    
    # 添加网格
    ax.grid(True, alpha=0.3, linestyle='--')
    
    # 添加图例
    ax.legend(loc='best', fontsize=10, framealpha=0.9)
    
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
    print(f"开源模型: {len(open_source)}")
    print(f"闭源模型: {len(closed_source)}")
    print(f"性能范围: {df['performance'].min():.1f} - {df['performance'].max():.1f}")
    print(f"平均性能: {df['performance'].mean():.2f}")

if __name__ == "__main__":
    plot_performance_vs_release()
