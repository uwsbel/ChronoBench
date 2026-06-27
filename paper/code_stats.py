#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Correct Token Calculation v3
============================
Using OFFICIAL category mapping from evaluatePy.py
"""

import os
import sys
import io
import tiktoken

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')

enc = tiktoken.get_encoding("cl100k_base")

def count_tokens(text):
    if not text:
        return 0
    return len(enc.encode(str(text)))

demo_data_path = "demo_data"

# OFFICIAL Category mapping from evaluatePy.py (99-110)
MBS_list = ["pendulum", "slider_crank", "gear", "mass_spring_damper", "particles"]
FEA_list = ["beam", "buckling", "rotor", "tablecloth", "cable"]
SEN_list = ["gps_imu", "lidar", "veh_app", "camera"]
RBT_list = ["turtlebot", "viper", "curiosity", "vehros", "sensros", "handler"]
VEH_list = ["citybus", "feda", "gator", "hmmwv", "kraz", "art", "rigid_highway", 
            "rigid_multipatches", "scm", "scm_hill", "uazbus", "m113", "sedan", "man"]

# Build category map
system_category_map = {}
for s in MBS_list:
    system_category_map[s] = 'MBS'
for s in FEA_list:
    system_category_map[s] = 'FEA'
for s in SEN_list:
    system_category_map[s] = 'Sensor'
for s in RBT_list:
    system_category_map[s] = 'Robot'
for s in VEH_list:
    system_category_map[s] = 'Vehicle'

# Verify all systems are mapped
systems = sorted([d for d in os.listdir(demo_data_path) if os.path.isdir(os.path.join(demo_data_path, d))])

print("=" * 80)
print("  SYSTEM CATEGORY VERIFICATION (OFFICIAL from evaluatePy.py)")
print("=" * 80)

unmapped = [s for s in systems if s not in system_category_map]
if unmapped:
    print(f"\n⚠️  WARNING: Unmapped systems: {unmapped}")
else:
    print("\n✓ All 34 systems are mapped!")

# Count by category
from collections import Counter
cat_systems = {}
for s in systems:
    cat = system_category_map.get(s, 'Other')
    if cat not in cat_systems:
        cat_systems[cat] = []
    cat_systems[cat].append(s)

print("\nSystems per category (OFFICIAL):")
total_systems = 0
for cat in ['MBS', 'FEA', 'Sensor', 'Robot', 'Vehicle', 'Other']:
    if cat in cat_systems:
        print(f"  {cat}: {len(cat_systems[cat])} systems")
        print(f"       {cat_systems[cat]}")
        total_systems += len(cat_systems[cat])

print(f"\nTotal: {total_systems} systems × 3 rounds = {total_systems * 3} tasks")

# Now calculate tokens
print("\n" + "=" * 80)
print("  TOKEN STATISTICS")
print("=" * 80)

all_data = []

for system in systems:
    system_path = os.path.join(demo_data_path, system)
    category = system_category_map.get(system, 'Other')
    
    for round_num in [1, 2, 3]:
        input_file = os.path.join(system_path, f"input{round_num}.txt")
        pyinput_file = os.path.join(system_path, f"pyinput{round_num}.py")
        truth_file = os.path.join(system_path, f"truth{round_num}.py")
        
        if not os.path.exists(input_file) or not os.path.exists(truth_file):
            continue
            
        with open(input_file, 'r', encoding='utf-8', errors='ignore') as f:
            input_text = f.read()
        input_tokens = count_tokens(input_text)
        
        code_tokens = 0
        if round_num > 1 and os.path.exists(pyinput_file):
            with open(pyinput_file, 'r', encoding='utf-8', errors='ignore') as f:
                code_text = f.read()
            code_tokens = count_tokens(code_text)
        
        total_prompt_tokens = input_tokens + code_tokens
        
        with open(truth_file, 'r', encoding='utf-8', errors='ignore') as f:
            solution_text = f.read()
        solution_tokens = count_tokens(solution_text)
        solution_lines = len(solution_text.strip().split('\n'))
        
        all_data.append({
            'system': system,
            'category': category,
            'round': round_num,
            'input_text_tokens': input_tokens,
            'code_context_tokens': code_tokens,
            'total_prompt_tokens': total_prompt_tokens,
            'solution_tokens': solution_tokens,
            'solution_lines': solution_lines
        })

print(f"\nTotal records: {len(all_data)}")

# By round
print("\n" + "-" * 60)
print("  BY ROUND")
print("-" * 60)

for round_num in [1, 2, 3]:
    round_data = [d for d in all_data if d['round'] == round_num]
    if not round_data:
        continue
    
    avg_text = sum(d['input_text_tokens'] for d in round_data) / len(round_data)
    avg_code = sum(d['code_context_tokens'] for d in round_data) / len(round_data)
    avg_prompt = sum(d['total_prompt_tokens'] for d in round_data) / len(round_data)
    avg_sol = sum(d['solution_tokens'] for d in round_data) / len(round_data)
    avg_lines = sum(d['solution_lines'] for d in round_data) / len(round_data)
    
    print(f"\nRound {round_num} ({len(round_data)} tasks):")
    print(f"  Text instruction:  {avg_text:>7.0f} tokens")
    print(f"  Code context:      {avg_code:>7.0f} tokens")
    print(f"  TOTAL PROMPT:      {avg_prompt:>7.0f} tokens")
    print(f"  Solution:          {avg_sol:>7.0f} tokens ({avg_lines:.0f} lines)")

# By category
print("\n" + "-" * 60)
print("  BY CATEGORY (OFFICIAL)")
print("-" * 60)

for cat in ['MBS', 'FEA', 'Sensor', 'Robot', 'Vehicle']:
    cat_data = [d for d in all_data if d['category'] == cat]
    if not cat_data:
        continue
    
    n_systems = len(set(d['system'] for d in cat_data))
    avg_prompt = sum(d['total_prompt_tokens'] for d in cat_data) / len(cat_data)
    avg_sol = sum(d['solution_tokens'] for d in cat_data) / len(cat_data)
    avg_lines = sum(d['solution_lines'] for d in cat_data) / len(cat_data)
    
    print(f"\n{cat} ({n_systems} systems, {len(cat_data)} tasks):")
    print(f"  Avg prompt:   {avg_prompt:>7.0f} tokens")
    print(f"  Avg solution: {avg_sol:>7.0f} tokens ({avg_lines:.0f} lines)")

# Overall
print("\n" + "-" * 60)
print("  OVERALL SUMMARY")
print("-" * 60)

avg_text = sum(d['input_text_tokens'] for d in all_data) / len(all_data)
avg_code = sum(d['code_context_tokens'] for d in all_data) / len(all_data)
avg_prompt = sum(d['total_prompt_tokens'] for d in all_data) / len(all_data)
avg_sol = sum(d['solution_tokens'] for d in all_data) / len(all_data)
avg_lines = sum(d['solution_lines'] for d in all_data) / len(all_data)

print(f"""
SimBench Dataset Statistics:
  Total systems:  {len(systems)}
  Total tasks:    {len(all_data)} (34 × 3 rounds)
  
  Average prompt:    {avg_prompt:>7.0f} tokens
    - Text only:     {avg_text:>7.0f} tokens
    - Code context:  {avg_code:>7.0f} tokens
  
  Average solution:  {avg_sol:>7.0f} tokens ({avg_lines:.0f} lines)
""")

# LaTeX table for paper
print("\n" + "=" * 80)
print("  LATEX TABLE FOR PAPER (OFFICIAL CATEGORIES)")
print("=" * 80)

print(r"""
\begin{table}[h!]
    \centering
    \caption{SimBench dataset statistics by category.}
    \label{tab:dataset_stats}
    \begin{tabular}{l r r r r}
        \toprule
        \textbf{Category} & \textbf{Systems} & \textbf{Tasks} & \textbf{Avg. Prompt} & \textbf{Avg. Solution} \\
        & & & \textbf{(tokens)} & \textbf{(tokens)} \\
        \midrule""")

for cat in ['MBS', 'FEA', 'Sensor', 'Robot', 'Vehicle']:
    cat_data = [d for d in all_data if d['category'] == cat]
    if not cat_data:
        continue
    n_systems = len(set(d['system'] for d in cat_data))
    avg_prompt = sum(d['total_prompt_tokens'] for d in cat_data) / len(cat_data)
    avg_sol = sum(d['solution_tokens'] for d in cat_data) / len(cat_data)
    print(f"        {cat} & {n_systems} & {len(cat_data)} & {avg_prompt:.0f} & {avg_sol:.0f} \\\\")

print(f"        \\midrule")
print(f"        \\textbf{{Total}} & \\textbf{{{len(systems)}}} & \\textbf{{{len(all_data)}}} & \\textbf{{{avg_prompt:.0f}}} & \\textbf{{{avg_sol:.0f}}} \\\\")
print(r"""        \bottomrule
    \end{tabular}
\end{table}
""")

print(r"""
\begin{table}[h!]
    \centering
    \caption{SimBench multi-turn prompt complexity.}
    \label{tab:turn_stats}
    \begin{tabular}{l r r r r r}
        \toprule
        \textbf{Turn} & \textbf{Tasks} & \textbf{Text} & \textbf{Code Context} & \textbf{Total Prompt} & \textbf{Solution} \\
        & & \textbf{(tokens)} & \textbf{(tokens)} & \textbf{(tokens)} & \textbf{(tokens)} \\
        \midrule""")

for round_num in [1, 2, 3]:
    round_data = [d for d in all_data if d['round'] == round_num]
    avg_text = sum(d['input_text_tokens'] for d in round_data) / len(round_data)
    avg_code = sum(d['code_context_tokens'] for d in round_data) / len(round_data)
    avg_prompt = sum(d['total_prompt_tokens'] for d in round_data) / len(round_data)
    avg_sol = sum(d['solution_tokens'] for d in round_data) / len(round_data)
    code_str = f"{avg_code:.0f}" if avg_code > 0 else "---"
    print(f"        Turn {round_num} & {len(round_data)} & {avg_text:.0f} & {code_str} & {avg_prompt:.0f} & {avg_sol:.0f} \\\\")

print(f"        \\midrule")
print(f"        \\textbf{{Average}} & \\textbf{{{len(all_data)}}} & \\textbf{{{avg_text:.0f}}} & \\textbf{{{avg_code:.0f}}} & \\textbf{{{avg_prompt:.0f}}} & \\textbf{{{avg_sol:.0f}}} \\\\")
print(r"""        \bottomrule
    \end{tabular}
\end{table}
""")

# ============================================================
# BENCHMARK COMPARISON
# ============================================================
print("\n" + "=" * 80)
print("  BENCHMARK COMPARISON")
print("=" * 80)

# Recalculate overall averages (to avoid using Round 3 values from loop)
avg_prompt_overall = sum(d['total_prompt_tokens'] for d in all_data) / len(all_data)
avg_sol_overall = sum(d['solution_tokens'] for d in all_data) / len(all_data)

# Other benchmarks data (from Hugging Face datasets analysis)
benchmarks = [
    ('MBPP', 500, 16, 58),
    ('MBPP+', 378, 19, 40),
    ('HumanEval', 164, 131, 54),
    ('HumanEval+', 164, 131, 54),
    ('DS-1000', 1000, 282, 42),
    ('BigCodeBench', 1140, 145, 112),
    ('CodeContests', 165, 593, 762),
]

print(f"\n{'Benchmark':<15} {'Tasks':>8} {'Prompt':>10} {'Solution':>10} {'P-Ratio':>10} {'S-Ratio':>10}")
print("-" * 75)

for name, tasks, prompt, solution in benchmarks:
    p_ratio = avg_prompt_overall / prompt if prompt > 0 else 0
    s_ratio = avg_sol_overall / solution if solution > 0 else 0
    print(f"{name:<15} {tasks:>8} {prompt:>10} {solution:>10} {p_ratio:>9.1f}x {s_ratio:>9.1f}x")

print("-" * 75)
print(f"{'SimBench':<15} {len(all_data):>8} {avg_prompt_overall:>10.0f} {avg_sol_overall:>10.0f} {'1.0x':>10} {'1.0x':>10}")

# Summary
avg_other_prompt = sum(b[2] for b in benchmarks) / len(benchmarks)
avg_other_solution = sum(b[3] for b in benchmarks) / len(benchmarks)

print(f"\n📊 Key Findings:")
print(f"   SimBench avg prompt:   {avg_prompt_overall:.0f} tokens")
print(f"   Other benchmarks avg:  {avg_other_prompt:.0f} tokens")
print(f"   SimBench is {avg_prompt_overall/avg_other_prompt:.1f}x longer in prompts")
print(f"\n   SimBench avg solution: {avg_sol_overall:.0f} tokens")
print(f"   Other benchmarks avg:  {avg_other_solution:.0f} tokens")
print(f"   SimBench is {avg_sol_overall/avg_other_solution:.1f}x longer in solutions")

# LaTeX table for benchmark comparison
print("\n" + "=" * 80)
print("  LATEX TABLE: BENCHMARK COMPARISON")
print("=" * 80)

print(r"""
\begin{table}[h!]
    \centering
    \caption{Comparison of SimBench with existing code generation benchmarks.}
    \label{tab:benchmark_comparison}
    \begin{tabular}{l r r r r r}
        \toprule
        \textbf{Benchmark} & \textbf{Tasks} & \textbf{Prompt} & \textbf{Solution} & \multicolumn{2}{c}{\textbf{Ratio to SimBench}} \\
        & & \textbf{(tokens)} & \textbf{(tokens)} & Prompt & Solution \\
        \midrule""")

for name, tasks, prompt, solution in benchmarks:
    p_ratio = avg_prompt_overall / prompt if prompt > 0 else 0
    s_ratio = avg_sol_overall / solution if solution > 0 else 0
    print(f"        {name} & {tasks} & {prompt} & {solution} & {p_ratio:.1f}$\\times$ & {s_ratio:.1f}$\\times$ \\\\")

print(f"        \\midrule")
print(f"        \\textbf{{SimBench (ours)}} & \\textbf{{{len(all_data)}}} & \\textbf{{{avg_prompt_overall:.0f}}} & \\textbf{{{avg_sol_overall:.0f}}} & 1.0$\\times$ & 1.0$\\times$ \\\\")
print(r"""        \bottomrule
    \end{tabular}
\end{table}
""")
