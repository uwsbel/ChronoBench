"""
过滤预训练模型：排除 prompt engineering (pe_前缀) 和 fine-tuned (f1, f3, sft1, lora1等后缀) 的模型
"""
import pandas as pd
import re

def is_pretrain_model(model_name):
    """
    判断是否为预训练模型
    
    排除条件：
    1. 以 pe_ 开头的模型（prompt engineering）
    2. 包含 -f1, -f3, -sft1, -lora1 等后缀的模型（fine-tuned）
    """
    # 排除 prompt engineering 模型
    if model_name.startswith('pe_'):
        return False
    
    # 排除 fine-tuned 模型（匹配 -f1, -f3, -sft1, -lora1 等模式）
    fine_tuned_patterns = [
        r'-f\d+',      # -f1, -f3 等
        r'-sft\d+',    # -sft1 等
        r'-lora\d+',   # -lora1 等
    ]
    
    for pattern in fine_tuned_patterns:
        if re.search(pattern, model_name):
            return False
    
    return True

def filter_pretrain_models(input_file, output_file):
    """
    从输入文件中过滤出预训练模型的数据
    """
    print(f"正在读取文件: {input_file}")
    df = pd.read_csv(input_file)
    
    print(f"原始数据行数: {len(df)}")
    print(f"原始模型数量: {df['model'].nunique()}")
    
    # 获取所有模型名称
    all_models = df['model'].unique()
    print(f"\n所有模型列表:")
    for model in sorted(all_models):
        status = "预训练" if is_pretrain_model(model) else "排除"
        print(f"  {model}: {status}")
    
    # 过滤预训练模型
    pretrain_mask = df['model'].apply(is_pretrain_model)
    df_pretrain = df[pretrain_mask].copy()
    
    print(f"\n过滤后数据行数: {len(df_pretrain)}")
    print(f"预训练模型数量: {df_pretrain['model'].nunique()}")
    
    print(f"\n预训练模型列表:")
    for model in sorted(df_pretrain['model'].unique()):
        print(f"  {model}")
    
    # 保存结果
    print(f"\n正在保存到: {output_file}")
    df_pretrain.to_csv(output_file, index=False)
    print("完成！")

if __name__ == "__main__":
    input_file = "scoring/out/all_metrics_merged.csv"
    output_file = "scoring/out/all_metrics_merged_pretrain_only.csv"
    
    filter_pretrain_models(input_file, output_file)
