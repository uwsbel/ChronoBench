#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SimBench Benchmark Analysis
===========================
Complete analysis of SimBench dataset statistics and comparison with other benchmarks.
Downloads benchmark data from Hugging Face and calculates all statistics.

Author: SimBench Team
"""

import os
import sys
import io
import json
import tiktoken
import pandas as pd
from collections import Counter

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')

# Tokenizer
enc = tiktoken.get_encoding("cl100k_base")

def count_tokens(text):
    if not text:
        return 0
    return len(enc.encode(str(text)))

def count_lines(text):
    if not text:
        return 0
    return len(str(text).strip().split('\n'))

# ============================================================
# PART 1: SIMBENCH STATISTICS
# ============================================================

def analyze_simbench():
    """Analyze SimBench dataset from local demo_data folder."""
    
    print("=" * 80)
    print("  PART 1: SIMBENCH DATASET ANALYSIS")
    print("=" * 80)
    
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
    for s in MBS_list: system_category_map[s] = 'MBS'
    for s in FEA_list: system_category_map[s] = 'FEA'
    for s in SEN_list: system_category_map[s] = 'Sensor'
    for s in RBT_list: system_category_map[s] = 'Robot'
    for s in VEH_list: system_category_map[s] = 'Vehicle'
    
    # Get systems
    systems = sorted([d for d in os.listdir(demo_data_path) if os.path.isdir(os.path.join(demo_data_path, d))])
    
    # Verify mapping
    unmapped = [s for s in systems if s not in system_category_map]
    if unmapped:
        print(f"\n⚠️  WARNING: Unmapped systems: {unmapped}")
    else:
        print(f"\n✓ All {len(systems)} systems are mapped!")
    
    # Count by category
    cat_systems = {}
    for s in systems:
        cat = system_category_map.get(s, 'Other')
        if cat not in cat_systems:
            cat_systems[cat] = []
        cat_systems[cat].append(s)
    
    print("\nSystems per category:")
    for cat in ['MBS', 'FEA', 'Sensor', 'Robot', 'Vehicle']:
        if cat in cat_systems:
            print(f"  {cat}: {len(cat_systems[cat])} systems - {cat_systems[cat]}")
    
    # Collect all data
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
    
    print(f"\nTotal tasks: {len(all_data)}")
    
    # By round
    print("\n" + "-" * 60)
    print("  BY ROUND")
    print("-" * 60)
    
    for round_num in [1, 2, 3]:
        round_data = [d for d in all_data if d['round'] == round_num]
        avg_text = sum(d['input_text_tokens'] for d in round_data) / len(round_data)
        avg_code = sum(d['code_context_tokens'] for d in round_data) / len(round_data)
        avg_prompt = sum(d['total_prompt_tokens'] for d in round_data) / len(round_data)
        avg_sol = sum(d['solution_tokens'] for d in round_data) / len(round_data)
        avg_lines = sum(d['solution_lines'] for d in round_data) / len(round_data)
        
        print(f"\nTurn {round_num} ({len(round_data)} tasks):")
        print(f"  Text: {avg_text:.0f} tok | Code Context: {avg_code:.0f} tok | Total Prompt: {avg_prompt:.0f} tok")
        print(f"  Solution: {avg_sol:.0f} tok ({avg_lines:.0f} lines)")
    
    # By category
    print("\n" + "-" * 60)
    print("  BY CATEGORY")
    print("-" * 60)
    
    for cat in ['MBS', 'FEA', 'Sensor', 'Robot', 'Vehicle']:
        cat_data = [d for d in all_data if d['category'] == cat]
        if not cat_data:
            continue
        n_systems = len(set(d['system'] for d in cat_data))
        avg_prompt = sum(d['total_prompt_tokens'] for d in cat_data) / len(cat_data)
        avg_sol = sum(d['solution_tokens'] for d in cat_data) / len(cat_data)
        print(f"  {cat}: {n_systems} systems, {len(cat_data)} tasks | Prompt: {avg_prompt:.0f} tok | Solution: {avg_sol:.0f} tok")
    
    # Overall
    avg_prompt = sum(d['total_prompt_tokens'] for d in all_data) / len(all_data)
    avg_sol = sum(d['solution_tokens'] for d in all_data) / len(all_data)
    avg_lines = sum(d['solution_lines'] for d in all_data) / len(all_data)
    min_sol = min(d['solution_tokens'] for d in all_data)
    max_sol = max(d['solution_tokens'] for d in all_data)
    
    print("\n" + "-" * 60)
    print("  OVERALL")
    print("-" * 60)
    print(f"\n  Total: {len(systems)} systems, {len(all_data)} tasks")
    print(f"  Avg Prompt:   {avg_prompt:.0f} tokens")
    print(f"  Avg Solution: {avg_sol:.0f} tokens ({avg_lines:.0f} lines)")
    print(f"  Solution Range: {min_sol} - {max_sol} tokens")
    
    return {
        'name': 'SimBench',
        'source': 'local (demo_data/)',
        'tasks': len(all_data),
        'avg_prompt_tokens': avg_prompt,
        'avg_solution_tokens': avg_sol,
        'avg_solution_lines': avg_lines,
        'min_solution': min_sol,
        'max_solution': max_sol,
        'all_data': all_data,
        'cat_systems': cat_systems
    }


# ============================================================
# PART 2: DOWNLOAD AND ANALYZE OTHER BENCHMARKS
# ============================================================

def analyze_other_benchmarks():
    """Download and analyze other code generation benchmarks from Hugging Face."""
    
    print("\n" + "=" * 80)
    print("  PART 2: OTHER BENCHMARKS (Downloaded from Hugging Face)")
    print("=" * 80)
    
    from datasets import load_dataset
    import warnings
    warnings.filterwarnings('ignore')
    
    results = []
    
    # 1. HumanEval
    print("\n[1] HumanEval...")
    try:
        ds = load_dataset("openai_humaneval", split="test")
        prompt_tokens = [count_tokens(item.get('prompt', '')) for item in ds]
        solution_tokens = [count_tokens(item.get('canonical_solution', '')) for item in ds]
        solution_lines = [count_lines(item.get('canonical_solution', '')) for item in ds]
        
        results.append({
            'name': 'HumanEval',
            'source': 'openai_humaneval',
            'tasks': len(ds),
            'avg_prompt_tokens': sum(prompt_tokens) / len(prompt_tokens),
            'avg_solution_tokens': sum(solution_tokens) / len(solution_tokens),
            'avg_solution_lines': sum(solution_lines) / len(solution_lines),
            'min_solution': min(solution_tokens),
            'max_solution': max(solution_tokens)
        })
        print(f"   ✓ {len(ds)} tasks | Prompt: {results[-1]['avg_prompt_tokens']:.0f} | Solution: {results[-1]['avg_solution_tokens']:.0f}")
    except Exception as e:
        print(f"   ✗ Error: {e}")
    
    # 2. MBPP
    print("\n[2] MBPP...")
    try:
        ds = load_dataset("mbpp", split="test")
        prompt_tokens = [count_tokens(item.get('text', '')) for item in ds]
        solution_tokens = [count_tokens(item.get('code', '')) for item in ds]
        solution_lines = [count_lines(item.get('code', '')) for item in ds]
        
        results.append({
            'name': 'MBPP',
            'source': 'mbpp',
            'tasks': len(ds),
            'avg_prompt_tokens': sum(prompt_tokens) / len(prompt_tokens),
            'avg_solution_tokens': sum(solution_tokens) / len(solution_tokens),
            'avg_solution_lines': sum(solution_lines) / len(solution_lines),
            'min_solution': min(solution_tokens),
            'max_solution': max(solution_tokens)
        })
        print(f"   ✓ {len(ds)} tasks | Prompt: {results[-1]['avg_prompt_tokens']:.0f} | Solution: {results[-1]['avg_solution_tokens']:.0f}")
    except Exception as e:
        print(f"   ✗ Error: {e}")
    
    # 3. HumanEval+
    print("\n[3] HumanEval+...")
    try:
        ds = load_dataset("evalplus/humanevalplus", split="test")
        prompt_tokens = [count_tokens(item.get('prompt', '')) for item in ds]
        solution_tokens = [count_tokens(item.get('canonical_solution', '')) for item in ds]
        solution_lines = [count_lines(item.get('canonical_solution', '')) for item in ds]
        
        results.append({
            'name': 'HumanEval+',
            'source': 'evalplus/humanevalplus',
            'tasks': len(ds),
            'avg_prompt_tokens': sum(prompt_tokens) / len(prompt_tokens),
            'avg_solution_tokens': sum(solution_tokens) / len(solution_tokens),
            'avg_solution_lines': sum(solution_lines) / len(solution_lines),
            'min_solution': min(solution_tokens),
            'max_solution': max(solution_tokens)
        })
        print(f"   ✓ {len(ds)} tasks | Prompt: {results[-1]['avg_prompt_tokens']:.0f} | Solution: {results[-1]['avg_solution_tokens']:.0f}")
    except Exception as e:
        print(f"   ✗ Error: {e}")
    
    # 4. MBPP+
    print("\n[4] MBPP+...")
    try:
        ds = load_dataset("evalplus/mbppplus", split="test")
        prompt_tokens = [count_tokens(item.get('prompt', '') or item.get('text', '')) for item in ds]
        solution_tokens = [count_tokens(item.get('canonical_solution', '') or item.get('code', '')) for item in ds]
        solution_lines = [count_lines(item.get('canonical_solution', '') or item.get('code', '')) for item in ds]
        
        results.append({
            'name': 'MBPP+',
            'source': 'evalplus/mbppplus',
            'tasks': len(ds),
            'avg_prompt_tokens': sum(prompt_tokens) / len(prompt_tokens),
            'avg_solution_tokens': sum(solution_tokens) / len(solution_tokens),
            'avg_solution_lines': sum(solution_lines) / len(solution_lines),
            'min_solution': min(solution_tokens),
            'max_solution': max(solution_tokens)
        })
        print(f"   ✓ {len(ds)} tasks | Prompt: {results[-1]['avg_prompt_tokens']:.0f} | Solution: {results[-1]['avg_solution_tokens']:.0f}")
    except Exception as e:
        print(f"   ✗ Error: {e}")
    
    # 5. DS-1000
    print("\n[5] DS-1000...")
    try:
        ds = load_dataset("xlangai/DS-1000", split="test")
        prompt_tokens = [count_tokens(item.get('prompt', '')) for item in ds]
        solution_tokens = [count_tokens(item.get('reference_code', '')) for item in ds]
        solution_lines = [count_lines(item.get('reference_code', '')) for item in ds]
        
        results.append({
            'name': 'DS-1000',
            'source': 'xlangai/DS-1000',
            'tasks': len(ds),
            'avg_prompt_tokens': sum(prompt_tokens) / len(prompt_tokens),
            'avg_solution_tokens': sum(solution_tokens) / len(solution_tokens),
            'avg_solution_lines': sum(solution_lines) / len(solution_lines),
            'min_solution': min(solution_tokens),
            'max_solution': max(solution_tokens)
        })
        print(f"   ✓ {len(ds)} tasks | Prompt: {results[-1]['avg_prompt_tokens']:.0f} | Solution: {results[-1]['avg_solution_tokens']:.0f}")
    except Exception as e:
        print(f"   ✗ Error: {e}")
    
    # 6. BigCodeBench
    print("\n[6] BigCodeBench...")
    try:
        ds = load_dataset("bigcode/bigcodebench", split="v0.1.2")
        prompt_tokens = [count_tokens(item.get('instruct_prompt', '') or item.get('complete_prompt', '')) for item in ds]
        solution_tokens = [count_tokens(item.get('canonical_solution', '')) for item in ds]
        solution_lines = [count_lines(item.get('canonical_solution', '')) for item in ds]
        
        results.append({
            'name': 'BigCodeBench',
            'source': 'bigcode/bigcodebench',
            'tasks': len(ds),
            'avg_prompt_tokens': sum(prompt_tokens) / len(prompt_tokens),
            'avg_solution_tokens': sum(solution_tokens) / len(solution_tokens),
            'avg_solution_lines': sum(solution_lines) / len(solution_lines),
            'min_solution': min(solution_tokens),
            'max_solution': max(solution_tokens)
        })
        print(f"   ✓ {len(ds)} tasks | Prompt: {results[-1]['avg_prompt_tokens']:.0f} | Solution: {results[-1]['avg_solution_tokens']:.0f}")
    except Exception as e:
        print(f"   ✗ Error: {e}")
    
    # 7. CodeContests
    print("\n[7] CodeContests...")
    try:
        ds = load_dataset("deepmind/code_contests", split="test", streaming=True)
        prompt_tokens, solution_tokens, solution_lines = [], [], []
        count = 0
        
        for item in ds:
            prompt_tokens.append(count_tokens(item.get('description', '')))
            sols = item.get('solutions', {})
            sol = sols.get('solution', [''])[0] if isinstance(sols, dict) else ''
            solution_tokens.append(count_tokens(sol))
            solution_lines.append(count_lines(sol))
            count += 1
            if count >= 165:
                break
        
        results.append({
            'name': 'CodeContests',
            'source': 'deepmind/code_contests',
            'tasks': count,
            'avg_prompt_tokens': sum(prompt_tokens) / len(prompt_tokens),
            'avg_solution_tokens': sum(solution_tokens) / len(solution_tokens),
            'avg_solution_lines': sum(solution_lines) / len(solution_lines),
            'min_solution': min(solution_tokens),
            'max_solution': max(solution_tokens)
        })
        print(f"   ✓ {count} tasks | Prompt: {results[-1]['avg_prompt_tokens']:.0f} | Solution: {results[-1]['avg_solution_tokens']:.0f}")
    except Exception as e:
        print(f"   ✗ Error: {e}")
    
    return results


# ============================================================
# PART 3: COMPARISON AND OUTPUT
# ============================================================

def generate_comparison(simbench_stats, other_benchmarks):
    """Generate comparison tables and LaTeX output."""
    
    print("\n" + "=" * 80)
    print("  PART 3: BENCHMARK COMPARISON")
    print("=" * 80)
    
    # Add SimBench to results
    all_benchmarks = other_benchmarks + [{
        'name': simbench_stats['name'],
        'source': simbench_stats['source'],
        'tasks': simbench_stats['tasks'],
        'avg_prompt_tokens': simbench_stats['avg_prompt_tokens'],
        'avg_solution_tokens': simbench_stats['avg_solution_tokens'],
        'avg_solution_lines': simbench_stats['avg_solution_lines'],
        'min_solution': simbench_stats['min_solution'],
        'max_solution': simbench_stats['max_solution']
    }]
    
    # Sort by solution tokens
    all_benchmarks.sort(key=lambda x: x['avg_solution_tokens'])
    
    # Print comparison table
    print(f"\n{'Benchmark':<15} {'Tasks':>7} {'Prompt':>9} {'Solution':>9} {'Lines':>7} {'Range':>15}")
    print("-" * 70)
    
    for b in all_benchmarks:
        marker = " ***" if b['name'] == 'SimBench' else ""
        range_str = f"{b['min_solution']}-{b['max_solution']}"
        print(f"{b['name']:<15} {b['tasks']:>7} {b['avg_prompt_tokens']:>9.0f} {b['avg_solution_tokens']:>9.0f} {b['avg_solution_lines']:>7.1f} {range_str:>15}{marker}")
    
    # Calculate ratios
    print("\n" + "-" * 70)
    print("  RATIO TO SIMBENCH")
    print("-" * 70)
    
    sb = simbench_stats
    
    print(f"\n{'Benchmark':<15} {'Prompt Ratio':>15} {'Solution Ratio':>15}")
    print("-" * 50)
    
    for b in other_benchmarks:
        p_ratio = sb['avg_prompt_tokens'] / b['avg_prompt_tokens']
        s_ratio = sb['avg_solution_tokens'] / b['avg_solution_tokens']
        print(f"{b['name']:<15} {p_ratio:>14.1f}x {s_ratio:>14.1f}x")
    
    # Average
    avg_other_prompt = sum(b['avg_prompt_tokens'] for b in other_benchmarks) / len(other_benchmarks)
    avg_other_solution = sum(b['avg_solution_tokens'] for b in other_benchmarks) / len(other_benchmarks)
    
    print("-" * 50)
    print(f"{'Average':<15} {sb['avg_prompt_tokens']/avg_other_prompt:>14.1f}x {sb['avg_solution_tokens']/avg_other_solution:>14.1f}x")
    
    # Key findings
    print("\n" + "=" * 80)
    print("  📊 KEY FINDINGS")
    print("=" * 80)
    print(f"""
  SimBench:
    • Avg Prompt:   {sb['avg_prompt_tokens']:.0f} tokens
    • Avg Solution: {sb['avg_solution_tokens']:.0f} tokens ({sb['avg_solution_lines']:.0f} lines)
  
  Other Benchmarks Average:
    • Avg Prompt:   {avg_other_prompt:.0f} tokens
    • Avg Solution: {avg_other_solution:.0f} tokens
  
  SimBench is:
    • {sb['avg_prompt_tokens']/avg_other_prompt:.1f}x longer in prompts
    • {sb['avg_solution_tokens']/avg_other_solution:.1f}x longer in solutions
""")
    
    # Generate LaTeX
    print("\n" + "=" * 80)
    print("  LATEX TABLES")
    print("=" * 80)
    
    # Table 1: Benchmark Comparison
    print(r"""
\begin{table}[h!]
    \centering
    \caption{Comparison of SimBench with existing code generation benchmarks. Token counts computed using tiktoken (cl100k\_base).}
    \label{tab:benchmark_comparison}
    \begin{tabular}{l r r r r r}
        \toprule
        \textbf{Benchmark} & \textbf{Tasks} & \textbf{Prompt} & \textbf{Solution} & \multicolumn{2}{c}{\textbf{SimBench Ratio}} \\
        & & \textbf{(tokens)} & \textbf{(tokens)} & Prompt & Solution \\
        \midrule""")
    
    for b in other_benchmarks:
        p_ratio = sb['avg_prompt_tokens'] / b['avg_prompt_tokens']
        s_ratio = sb['avg_solution_tokens'] / b['avg_solution_tokens']
        print(f"        {b['name']} & {b['tasks']} & {b['avg_prompt_tokens']:.0f} & {b['avg_solution_tokens']:.0f} & {p_ratio:.1f}$\\times$ & {s_ratio:.1f}$\\times$ \\\\")
    
    print(f"        \\midrule")
    print(f"        \\textbf{{SimBench (ours)}} & \\textbf{{{sb['tasks']}}} & \\textbf{{{sb['avg_prompt_tokens']:.0f}}} & \\textbf{{{sb['avg_solution_tokens']:.0f}}} & 1.0$\\times$ & 1.0$\\times$ \\\\")
    print(r"""        \bottomrule
    \end{tabular}
\end{table}
""")
    
    # Table 2: SimBench Category Stats
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
    
    all_data = simbench_stats['all_data']
    for cat in ['MBS', 'FEA', 'Sensor', 'Robot', 'Vehicle']:
        cat_data = [d for d in all_data if d['category'] == cat]
        if not cat_data:
            continue
        n_systems = len(set(d['system'] for d in cat_data))
        avg_prompt = sum(d['total_prompt_tokens'] for d in cat_data) / len(cat_data)
        avg_sol = sum(d['solution_tokens'] for d in cat_data) / len(cat_data)
        print(f"        {cat} & {n_systems} & {len(cat_data)} & {avg_prompt:.0f} & {avg_sol:.0f} \\\\")
    
    print(f"        \\midrule")
    print(f"        \\textbf{{Total}} & \\textbf{{{len(simbench_stats['cat_systems']['MBS']) + len(simbench_stats['cat_systems']['FEA']) + len(simbench_stats['cat_systems']['Sensor']) + len(simbench_stats['cat_systems']['Robot']) + len(simbench_stats['cat_systems']['Vehicle'])}}} & \\textbf{{{len(all_data)}}} & \\textbf{{{sb['avg_prompt_tokens']:.0f}}} & \\textbf{{{sb['avg_solution_tokens']:.0f}}} \\\\")
    print(r"""        \bottomrule
    \end{tabular}
\end{table}
""")
    
    # Table 3: Turn Complexity
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
    
    avg_text = sum(d['input_text_tokens'] for d in all_data) / len(all_data)
    avg_code = sum(d['code_context_tokens'] for d in all_data) / len(all_data)
    print(f"        \\midrule")
    print(f"        \\textbf{{Average}} & \\textbf{{{len(all_data)}}} & \\textbf{{{avg_text:.0f}}} & \\textbf{{{avg_code:.0f}}} & \\textbf{{{sb['avg_prompt_tokens']:.0f}}} & \\textbf{{{sb['avg_solution_tokens']:.0f}}} \\\\")
    print(r"""        \bottomrule
    \end{tabular}
\end{table}
""")
    
    # Save to CSV
    df = pd.DataFrame(all_benchmarks)
    os.makedirs('paper/out', exist_ok=True)
    df.to_csv('paper/out/benchmark_comparison_final.csv', index=False)
    print(f"\n✓ Saved to: paper/out/benchmark_comparison_final.csv")
    
    return all_benchmarks


# ============================================================
# MAIN
# ============================================================

if __name__ == "__main__":
    # Part 1: Analyze SimBench
    simbench_stats = analyze_simbench()
    
    # Part 2: Download and analyze other benchmarks
    other_benchmarks = analyze_other_benchmarks()
    
    # Part 3: Generate comparison
    all_benchmarks = generate_comparison(simbench_stats, other_benchmarks)
    
    print("\n" + "=" * 80)
    print("  DONE!")
    print("=" * 80)
