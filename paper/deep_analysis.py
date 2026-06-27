#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Deep Analysis for SimBench Paper
Finding interesting patterns and examples
"""

import sys
import io
import pandas as pd
import numpy as np

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')

# Load data
df = pd.read_csv('paper/out/all_metrics_merged.csv')
df['score_ref_doc_norm'] = df['score_reference_document'] / 100

# System categories
MBS = ['pendulum', 'slider_crank', 'gear', 'mass_spring_damper', 'particles']
FEA = ['beam', 'buckling', 'rotor', 'tablecloth', 'cable']
SEN = ['gps_imu', 'lidar', 'veh_app', 'camera']
RBT = ['turtlebot', 'viper', 'curiosity', 'vehros', 'sensros', 'handler']
VEH = ['citybus', 'feda', 'gator', 'hmmwv', 'kraz', 'art', 'rigid_highway', 
       'rigid_multipatches', 'scm', 'scm_hill', 'uazbus', 'm113', 'sedan', 'man']

cat_map = {}
for s in MBS: cat_map[s] = 'MBS'
for s in FEA: cat_map[s] = 'FEA'
for s in SEN: cat_map[s] = 'Sensor'
for s in RBT: cat_map[s] = 'Robot'
for s in VEH: cat_map[s] = 'Vehicle'

df['category'] = df['system'].map(cat_map)

# Training type
def get_type(m):
    if m.lower().startswith('pe_'):
        return 'PE'
    elif any(x in m.lower() for x in ['-f1', '-f3', 'lora', 'sft']):
        return 'FT'
    return 'Base'

df['type'] = df['model'].apply(get_type)

print("=" * 70)
print("  DEEP ANALYSIS FOR SIMBENCH PAPER")
print("=" * 70)

# ============================================================
# 1. CodeBLEU vs LLM Judge Disagreement
# ============================================================
print("\n" + "=" * 70)
print("  1. METRIC DISAGREEMENT ANALYSIS")
print("=" * 70)

model_avg = df.groupby('model').agg({
    'codebleu': 'mean',
    'score_ref_doc_norm': 'mean'
})
model_avg['gap'] = model_avg['score_ref_doc_norm'] - model_avg['codebleu']

print("\n  A. High LLM-Judge but Low CodeBLEU (Novel/Creative Solutions):")
print("     These models generate correct code but in a different style")
for model, row in model_avg.nlargest(5, 'gap').iterrows():
    print(f"     {model:35s} CodeBLEU={row['codebleu']:.3f} LLM={row['score_ref_doc_norm']:.3f} Gap={row['gap']:+.3f}")

print("\n  B. High CodeBLEU but Low LLM-Judge (Shallow Copy):")
print("     These models copy reference style but miss functional correctness")
for model, row in model_avg.nsmallest(5, 'gap').iterrows():
    print(f"     {model:35s} CodeBLEU={row['codebleu']:.3f} LLM={row['score_ref_doc_norm']:.3f} Gap={row['gap']:+.3f}")

# ============================================================
# 2. Round Dynamics
# ============================================================
print("\n" + "=" * 70)
print("  2. MULTI-ROUND LEARNING DYNAMICS")
print("=" * 70)

round_perf = df.groupby(['model', 'round'])['score_ref_doc_norm'].mean().unstack()
round_perf.columns = ['R1', 'R2', 'R3']
round_perf['R1_to_R2'] = round_perf['R2'] - round_perf['R1']
round_perf['R2_to_R3'] = round_perf['R3'] - round_perf['R2']
round_perf['Total'] = round_perf['R3'] - round_perf['R1']

print("\n  A. Best Learners (largest R1→R3 improvement):")
for model, row in round_perf.nlargest(5, 'Total').iterrows():
    print(f"     {model:35s} R1={row['R1']:.3f} → R3={row['R3']:.3f} (+{row['Total']:.3f})")

print("\n  B. Models that DEGRADE from R2 to R3 (over-correction?):")
degrade = round_perf[round_perf['R2_to_R3'] < -0.05].nsmallest(5, 'R2_to_R3')
for model, row in degrade.iterrows():
    print(f"     {model:35s} R2={row['R2']:.3f} → R3={row['R3']:.3f} ({row['R2_to_R3']:+.3f})")

print("\n  C. Models with Consistent Improvement (R1→R2→R3 all positive):")
consistent = round_perf[(round_perf['R1_to_R2'] > 0.1) & (round_perf['R2_to_R3'] > 0)]
consistent = consistent.nlargest(5, 'Total')
for model, row in consistent.iterrows():
    print(f"     {model:35s} {row['R1']:.3f} → {row['R2']:.3f} → {row['R3']:.3f}")

# ============================================================
# 3. Specialist vs Generalist
# ============================================================
print("\n" + "=" * 70)
print("  3. SPECIALIST vs GENERALIST MODELS")
print("=" * 70)

cat_perf = df.groupby(['model', 'category'])['score_ref_doc_norm'].mean().unstack()
cats = ['FEA', 'MBS', 'Robot', 'Sensor', 'Vehicle']
cat_perf['variance'] = cat_perf[cats].var(axis=1)
cat_perf['mean_score'] = cat_perf[cats].mean(axis=1)
cat_perf['best_cat'] = cat_perf[cats].idxmax(axis=1)
cat_perf['worst_cat'] = cat_perf[cats].idxmin(axis=1)

print("\n  A. Generalists (consistent across categories, low variance):")
for model, row in cat_perf.nsmallest(5, 'variance').iterrows():
    print(f"     {model:35s} var={row['variance']:.4f} mean={row['mean_score']:.3f}")

print("\n  B. Specialists (high variance, excel at specific domains):")
for model, row in cat_perf.nlargest(5, 'variance').iterrows():
    best = row['best_cat']
    worst = row['worst_cat']
    print(f"     {model:35s} var={row['variance']:.4f} best={best}({row[best]:.3f}) worst={worst}({row[worst]:.3f})")

# ============================================================
# 4. Hardest/Easiest System Analysis
# ============================================================
print("\n" + "=" * 70)
print("  4. SYSTEM DIFFICULTY DEEP DIVE")
print("=" * 70)

sys_perf = df.groupby('system').agg({
    'score_ref_doc_norm': 'mean',
    'codebleu': 'mean',
    'category': 'first'
})

print("\n  A. Hardest Systems (where even best models struggle):")
hardest = sys_perf.nsmallest(5, 'score_ref_doc_norm')
for sys, row in hardest.iterrows():
    # Find best model for this system
    best_model = df[df['system'] == sys].groupby('model')['score_ref_doc_norm'].mean().idxmax()
    best_score = df[df['system'] == sys].groupby('model')['score_ref_doc_norm'].mean().max()
    print(f"     {sys:25s} ({row['category']:8s}) avg={row['score_ref_doc_norm']:.3f}, best={best_model}({best_score:.3f})")

print("\n  B. Easiest Systems (high average scores):")
easiest = sys_perf.nlargest(5, 'score_ref_doc_norm')
for sys, row in easiest.iterrows():
    print(f"     {sys:25s} ({row['category']:8s}) avg={row['score_ref_doc_norm']:.3f}")

# ============================================================
# 5. Per-System Model Ranking Consistency
# ============================================================
print("\n" + "=" * 70)
print("  5. RANKING CONSISTENCY ACROSS SYSTEMS")
print("=" * 70)

# For each system, rank models
from scipy.stats import spearmanr

systems = df['system'].unique()
models = df['model'].unique()

# Create model-system performance matrix
perf_matrix = df.pivot_table(values='score_ref_doc_norm', index='model', columns='system', aggfunc='mean')

# Calculate rank correlation between systems
print("\n  A. Systems with SIMILAR difficulty patterns (high correlation):")
print("     (models that do well on one also do well on the other)")

# Sample a few interesting correlations
sample_systems = ['pendulum', 'lidar', 'art', 'beam', 'turtlebot']
for i, s1 in enumerate(sample_systems):
    for s2 in sample_systems[i+1:]:
        if s1 in perf_matrix.columns and s2 in perf_matrix.columns:
            valid = perf_matrix[[s1, s2]].dropna()
            if len(valid) > 5:
                rho, _ = spearmanr(valid[s1], valid[s2])
                if abs(rho) > 0.6:
                    print(f"     {s1} ↔ {s2}: ρ={rho:.3f}")

# ============================================================
# 6. Fine-tuning Case Studies
# ============================================================
print("\n" + "=" * 70)
print("  6. FINE-TUNING CASE STUDIES")
print("=" * 70)

ft_models = df[df['type'] == 'FT']['model'].unique()
print(f"\n  Fine-tuned models: {list(ft_models)}")

# Compare gpt-4o-mini variants
gpt_mini_base = df[df['model'] == 'gpt-4o-mini'].groupby('system')['score_ref_doc_norm'].mean()
gpt_mini_f1 = df[df['model'] == 'gpt-4o-mini-f1'].groupby('system')['score_ref_doc_norm'].mean()
gpt_mini_f3 = df[df['model'] == 'gpt-4o-mini-f3'].groupby('system')['score_ref_doc_norm'].mean()

print("\n  A. gpt-4o-mini Fine-tuning: Per-System Improvement")
common = gpt_mini_base.index.intersection(gpt_mini_f3.index)
improvements = (gpt_mini_f3[common] - gpt_mini_base[common]).sort_values(ascending=False)

print("     Biggest improvements:")
for sys, imp in improvements.head(5).items():
    print(f"       {sys:25s} +{imp:.3f} ({gpt_mini_base[sys]:.3f} → {gpt_mini_f3[sys]:.3f})")

print("     Smallest improvements (or degradation):")
for sys, imp in improvements.tail(5).items():
    print(f"       {sys:25s} {imp:+.3f} ({gpt_mini_base[sys]:.3f} → {gpt_mini_f3[sys]:.3f})")

# ============================================================
# 7. o3/o4-mini Reasoning Model Analysis
# ============================================================
print("\n" + "=" * 70)
print("  7. REASONING MODELS (o3, o4-mini) ANALYSIS")
print("=" * 70)

reasoning_models = ['o3', 'o4-mini']
for model in reasoning_models:
    if model in df['model'].values:
        model_data = df[df['model'] == model]
        avg_llm = model_data['score_ref_doc_norm'].mean()
        avg_code = model_data['codebleu'].mean()
        
        print(f"\n  {model}:")
        print(f"     LLM Judge: {avg_llm:.3f}, CodeBLEU: {avg_code:.3f}")
        print(f"     Gap (LLM - CodeBLEU): {avg_llm - avg_code:+.3f}")
        
        # Best and worst categories
        cat_scores = model_data.groupby('category')['score_ref_doc_norm'].mean()
        print(f"     Best category: {cat_scores.idxmax()} ({cat_scores.max():.3f})")
        print(f"     Worst category: {cat_scores.idxmin()} ({cat_scores.min():.3f})")

print("\n" + "=" * 70)
print("  ANALYSIS COMPLETE")
print("=" * 70)
