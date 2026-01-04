#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Check actual dataset statistics"""

import sys
import io
import pandas as pd

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')

# Load data
df = pd.read_csv('output_llms/combined_evaluation_scores.csv')
df = df.rename(columns={'Test Model': 'Model'})

print("=" * 60)
print("  ACTUAL DATASET STATISTICS")
print("=" * 60)

# System categories from evaluatePy.py
system_category_map = {
    'art': 'MBS', 'beam': 'FEA', 'buckling': 'FEA', 'cable': 'FEA',
    'camera': 'Sensor', 'citybus': 'Vehicle', 'curiosity': 'Robot',
    'feda': 'Vehicle', 'gator': 'Vehicle', 'gear': 'MBS',
    'gps_imu': 'Sensor', 'handler': 'Robot', 'hmmwv': 'Vehicle',
    'kraz': 'Vehicle', 'lidar': 'Sensor', 'm113': 'Vehicle',
    'man': 'Vehicle', 'mass_spring_damper': 'MBS', 'particles': 'MBS',
    'pendulum': 'MBS', 'rigid_highway': 'MBS', 'rigid_multipatches': 'MBS',
    'rotor': 'MBS', 'scm': 'Vehicle', 'scm_hill': 'Vehicle',
    'sedan': 'Vehicle', 'sensros': 'Sensor', 'slider_crank': 'MBS',
    'tablecloth': 'FEA', 'turtlebot': 'Robot', 'uazbus': 'Vehicle',
    'veh_app': 'Vehicle', 'vehros': 'Robot', 'viper': 'Robot'
}

# Get unique systems
systems = df['System'].unique()
print(f"\nTotal unique systems: {len(systems)}")

# Count by category
category_systems = {}
for system in systems:
    cat = system_category_map.get(system, 'Unknown')
    if cat not in category_systems:
        category_systems[cat] = []
    category_systems[cat].append(system)

print("\n--- Systems per Category ---")
for cat, sys_list in sorted(category_systems.items()):
    print(f"{cat}: {len(sys_list)} systems")
    for s in sorted(sys_list):
        print(f"    - {s}")

# Rounds
rounds = df['Round'].unique()
print(f"\n--- Rounds ---")
print(f"Rounds: {list(rounds)}")
print(f"Number of rounds: {len(rounds)}")

# Total tasks
tasks = df.groupby(['System', 'Round']).size()
print(f"\n--- Total unique (System, Round) combinations: {len(tasks)} ---")

# Summary table
print("\n" + "=" * 60)
print("  SUMMARY TABLE FOR PAPER")
print("=" * 60)
print(f"\n{'Category':<10} {'Systems':<10} {'Tasks':<10}")
print("-" * 30)
total_sys = 0
total_tasks = 0
for cat in ['MBS', 'FEA', 'Vehicle', 'Sensor', 'Robot']:
    n_sys = len(category_systems.get(cat, []))
    n_tasks = n_sys * 3
    total_sys += n_sys
    total_tasks += n_tasks
    print(f"{cat:<10} {n_sys:<10} {n_tasks:<10}")
print("-" * 30)
print(f"{'Total':<10} {total_sys:<10} {total_tasks:<10}")

# Models
print("\n" + "=" * 60)
print("  MODEL STATISTICS")
print("=" * 60)
models = df['Model'].unique()
print(f"\nTotal unique models: {len(models)}")

# Base vs fine-tuned
def is_base(name):
    n = name.lower()
    return not any(x in n for x in ['_f1', '_f3', '_lora', '_sft', 'pe_', '-f1', '-f3'])

base_models = [m for m in models if is_base(m)]
ft_models = [m for m in models if not is_base(m)]
print(f"Base models: {len(base_models)}")
print(f"Fine-tuned/PE models: {len(ft_models)}")
