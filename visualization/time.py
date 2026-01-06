
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import matplotlib.dates as mdates
from datetime import datetime
import numpy as np

# --- 1. 数据准备 ---
# 手动从 LaTeX 表格中提取的数据
data = [
    {"Model": "claude-4-sonnet-20250514", "Company": "Anthropic", "Open": False, "Release": "2025-Q2", "Perf": 49},
    {"Model": "o3", "Company": "OpenAI", "Open": False, "Release": "2025-Q2", "Perf": 46},
    {"Model": "claude-3-7-sonnet-20250219", "Company": "Anthropic", "Open": False, "Release": "2025-Q1", "Perf": 43},
    {"Model": "o4-mini", "Company": "OpenAI", "Open": False, "Release": "2025-Q2", "Perf": 42},
    {"Model": "qwen3-235b-a22b", "Company": "Alibaba", "Open": True, "Release": "2025-Q2", "Perf": 42},
    {"Model": "Gemini-2.5-pro", "Company": "Google", "Open": False, "Release": "2025-Q2", "Perf": 41},
    {"Model": "gpt-4.1-mini", "Company": "OpenAI", "Open": False, "Release": "2025-Q2", "Perf": 41},
    {"Model": "gpt-4o-mini", "Company": "OpenAI", "Open": False, "Release": "2024-Q3", "Perf": 41},
    {"Model": "llama4-maverick", "Company": "Meta", "Open": True, "Release": "2025-Q2", "Perf": 41},
    {"Model": "llama4-scout", "Company": "Meta", "Open": True, "Release": "2025-Q2", "Perf": 41},
    {"Model": "llama-3.3-70b", "Company": "Meta", "Open": True, "Release": "2024-Q4", "Perf": 41},
    {"Model": "deepseek-r1-32b", "Company": "DeepSeek", "Open": True, "Release": "2025-Q1", "Perf": 40},
    {"Model": "gpt-4.1-nano", "Company": "OpenAI", "Open": False, "Release": "2025-Q2", "Perf": 40},
    {"Model": "llama-3.1-70b", "Company": "Meta", "Open": True, "Release": "2024-Q3", "Perf": 40},
    {"Model": "gpt-4.1", "Company": "OpenAI", "Open": False, "Release": "2025-Q2", "Perf": 39},
    {"Model": "Gemini-1.5-pro", "Company": "Google", "Open": False, "Release": "2024-Q1", "Perf": 39},
    {"Model": "codestral-22b-v0.1", "Company": "Mistral", "Open": True, "Release": "2024-Q2", "Perf": 39},
    {"Model": "llama-3.1-405b", "Company": "Meta", "Open": True, "Release": "2024-Q3", "Perf": 39},
    {"Model": "mixtral-8x22b-v0.1", "Company": "Mistral", "Open": True, "Release": "2024-Q2", "Perf": 39},
    {"Model": "llama-3.1-8b", "Company": "Meta", "Open": True, "Release": "2024-Q3", "Perf": 38},
    {"Model": "mistral-nemo-12b", "Company": "Mistral", "Open": True, "Release": "2024-Q3", "Perf": 38},
    {"Model": "gemma-2-27b", "Company": "Google", "Open": True, "Release": "2024-Q2", "Perf": 36},
    {"Model": "mixtral-8x7b-v0.1", "Company": "Mistral", "Open": True, "Release": "2023-Q4", "Perf": 36},
    {"Model": "claude-3-5-sonnet", "Company": "Anthropic", "Open": False, "Release": "2024-Q2", "Perf": 34},
    {"Model": "deepseek-r1-8b", "Company": "DeepSeek", "Open": True, "Release": "2025-Q1", "Perf": 34},
    {"Model": "gpt-4o", "Company": "OpenAI", "Open": False, "Release": "2024-Q2", "Perf": 34},
    {"Model": "nemotron-4-340b", "Company": "NVIDIA", "Open": True, "Release": "2024-Q2", "Perf": 34},
    {"Model": "gemma-2-9b", "Company": "Google", "Open": True, "Release": "2024-Q2", "Perf": 32},
    {"Model": "gemma-2-2b", "Company": "Google", "Open": True, "Release": "2024-Q2", "Perf": 30},
    {"Model": "mamba-codestral-7b-v0.1", "Company": "Mistral", "Open": True, "Release": "2024-Q3", "Perf": 30},
    {"Model": "gemma-3-1b", "Company": "Google", "Open": True, "Release": "2025-Q1", "Perf": 26},
    {"Model": "phi-3-mini-128k", "Company": "Microsoft", "Open": True, "Release": "2024-Q2", "Perf": 25},
    {"Model": "phi-3-medium-128k", "Company": "Microsoft", "Open": True, "Release": "2024-Q2", "Perf": 21},
]

df = pd.DataFrame(data)

# --- 2. 数据清洗与转换 ---
# 将布尔值转换为易读的标签
df['Type'] = df['Open'].apply(lambda x: 'Open Source (开源)' if x else 'Closed Source (闭源)')

# 定义函数将季度字符串 (YYYY-QN) 转换为 datetime 对象
def quarter_to_date(q_str):
    year, q = q_str.split('-Q')
    # 将季度映射到该季度的第一个月
    month = (int(q) - 1) * 3 + 1 
    return datetime(int(year), month, 1)

df['Date_Raw'] = df['Release'].apply(quarter_to_date)

# 添加一点随机抖动 (Jitter) 到日期上，防止同一个月发布的点完全重叠
# 将日期转换为数字便于计算
date_num = mdates.date2num(df['Date_Raw'])
# 添加最多 +/- 15天的随机抖动
jitter = np.random.uniform(-15, 15, size=len(df))
df['Date_Jittered'] = mdates.num2date(date_num + jitter)


# --- 3. 绘图设置 ---
sns.set_theme(style="whitegrid", font_scale=1.1)
plt.figure(figsize=(14, 9))

# 创建散点图
# hue=Company: 用颜色区分公司
# style=Type: 用形状区分是否开源
scatter = sns.scatterplot(
    data=df,
    x="Date_Jittered",
    y="Perf",
    hue="Company",
    style="Type",
    markers={"Open Source (开源)": "X", "Closed Source (闭源)": "o"},
    s=180,          # 点的大小
    alpha=0.8,      # 透明度
    palette="tab10", # 颜色板
    edgecolor="k"   # 给点加上黑色边框提高对比度
)

# --- 4. 图表美化和标签 ---
plt.title('LLM Performance vs. Release Time\n(Based on J-LLM-Ref-Doc Score)', fontsize=18, pad=20, fontweight='bold')
plt.ylabel('Performance Score (Perf)', fontsize=14, labelpad=15)
plt.xlabel('Release Date (Quarter)', fontsize=14, labelpad=15)

# 设置 X 轴格式
ax = plt.gca()
# 主刻度设为每季度首月
ax.xaxis.set_major_locator(mdates.MonthLocator(bymonth=[1, 4, 7, 10]))
# 格式化显示为 年-月
ax.xaxis.set_major_formatter(mdates.DateFormatter('%Y-%m'))
plt.xticks(rotation=45)

# 设置 Y 轴范围，稍微留点空间
plt.ylim(18, 52)

# 优化图例
# 将图例移到图外侧，防止遮挡数据点
plt.legend(bbox_to_anchor=(1.02, 1), loc='upper left', borderaxespad=0., title='Model Information', frameon=True, shadow=True)

# 添加网格线辅助阅读
plt.grid(True, which='major', linestyle='--', linewidth=0.5, color='grey')

# 标注最高分模型
top_model = df.loc[df['Perf'].idxmax()]
plt.annotate(f"Highest: {top_model['Model']}",
             xy=(top_model['Date_Jittered'], top_model['Perf']),
             xytext=(-20, 15), textcoords='offset points',
             arrowprops=dict(arrowstyle="->", color='black', lw=1.5),
             fontweight='bold')

plt.tight_layout()

# 保存或显示图表
# plt.savefig('llm_perf_trend.png', dpi=300, bbox_inches='tight')
plt.show()
