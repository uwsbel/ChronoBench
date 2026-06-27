"""
从预训练模型数据中提取高级指标
只保留：codebleu, rougeLsum, score_document, score_reference, score_reference_document
"""
import pandas as pd

def filter_advanced_metrics(input_file, output_file):
    """
    从输入文件中提取高级指标列
    """
    print(f"正在读取文件: {input_file}")
    df = pd.read_csv(input_file)
    
    print(f"原始数据行数: {len(df)}")
    print(f"原始列数: {len(df.columns)}")
    print(f"原始列名: {list(df.columns)}")
    
    # 定义要保留的列
    identifier_cols = ['model', 'system', 'round']  # 标识列
    advanced_metrics = [
        'codebleu',
        'rougeLsum',
        'score_document',
        'score_reference',
        'score_reference_document'
    ]
    
    # 检查所有需要的列是否存在
    all_cols = identifier_cols + advanced_metrics
    missing_cols = [col for col in all_cols if col not in df.columns]
    
    if missing_cols:
        print(f"\n警告：以下列不存在: {missing_cols}")
        print("可用的列:", list(df.columns))
        return
    
    # 选择要保留的列
    df_filtered = df[all_cols].copy()
    
    print(f"\n过滤后列数: {len(df_filtered.columns)}")
    print(f"保留的列: {list(df_filtered.columns)}")
    print(f"数据行数: {len(df_filtered)}")
    
    # 保存结果
    print(f"\n正在保存到: {output_file}")
    df_filtered.to_csv(output_file, index=False)
    print("完成！")
    
    # 显示前几行作为预览
    print(f"\n前5行预览:")
    print(df_filtered.head())

if __name__ == "__main__":
    input_file = "paper/out/all_metrics_merged_pretrain_only.csv"
    output_file = "paper/out/all_metrics_merged_pretrain_only_advanced.csv"
    
    filter_advanced_metrics(input_file, output_file)
