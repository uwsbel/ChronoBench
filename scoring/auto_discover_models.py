#!/usr/bin/env python3
"""
Automatically discover all models that have output directories.
This ensures no model is missed in the scoring pipeline.
"""

import os
import json
from pathlib import Path

def discover_models():
    """
    Discover all models with output directories.
    
    Returns:
        list: List of model names that have output directories
    """
    output_dir = Path("/home/hongyu/Documents/SimBench/output_llms")
    models = []
    
    if output_dir.exists():
        for item in sorted(output_dir.iterdir()):
            if item.is_dir() and not item.name.startswith('.'):
                # Check if it has actual content (at least one system folder)
                has_content = False
                for subitem in item.iterdir():
                    if subitem.is_dir() and not subitem.name.startswith('.'):
                        has_content = True
                        break
                
                if has_content:
                    models.append(item.name)
    
    return models

def categorize_models(models):
    """
    Categorize models by their type/family.
    
    Args:
        models: List of model names
    
    Returns:
        dict: Categorized model names
    """
    categories = {
        'llama_family': [],
        'gpt_family': [],
        'gemma_family': [],
        'deepseek_family': [],
        'claude_family': [],
        'gemini_family': [],
        'mistral_family': [],
        'phi_family': [],
        'prompt_engineering': [],
        'other': []
    }
    
    for model in models:
        model_lower = model.lower()
        
        if model.startswith('pe_'):
            categories['prompt_engineering'].append(model)
        elif 'llama' in model_lower:
            categories['llama_family'].append(model)
        elif 'gpt' in model_lower or model in ['o3', 'o4-mini']:
            categories['gpt_family'].append(model)
        elif 'gemma' in model_lower:
            categories['gemma_family'].append(model)
        elif 'deepseek' in model_lower:
            categories['deepseek_family'].append(model)
        elif 'claude' in model_lower:
            categories['claude_family'].append(model)
        elif 'gemini' in model_lower.replace('-', ''):
            categories['gemini_family'].append(model)
        elif 'mistral' in model_lower or 'mixtral' in model_lower:
            categories['mistral_family'].append(model)
        elif 'phi' in model_lower:
            categories['phi_family'].append(model)
        else:
            categories['other'].append(model)
    
    # Remove empty categories
    return {k: v for k, v in categories.items() if v}

def export_model_list(output_file="discovered_models.json"):
    """
    Export discovered models to a JSON file.
    
    Args:
        output_file: Path to output JSON file
    """
    models = discover_models()
    categorized = categorize_models(models)
    
    result = {
        'total_models': len(models),
        'all_models': models,
        'categorized': categorized
    }
    
    output_path = Path(__file__).parent / output_file
    with open(output_path, 'w') as f:
        json.dump(result, f, indent=2)
    
    print(f"Discovered {len(models)} models")
    print(f"Exported to: {output_path}")
    
    return result

def generate_python_list():
    """
    Generate a Python list string for p_sim_score.py
    
    Returns:
        str: Python code for the model list
    """
    models = discover_models()
    categorized = categorize_models(models)
    
    lines = ["# Automatically discovered models from output directories\n"]
    lines.append("test_model_list = [\n")
    
    for category, model_list in categorized.items():
        if model_list:
            lines.append(f"    # {category.replace('_', ' ').title()}\n")
            for model in model_list:
                lines.append(f'    "{model}",\n')
            lines.append("\n")
    
    # Remove last newline and comma
    if lines[-1] == "\n":
        lines = lines[:-1]
    if lines[-1].endswith(",\n"):
        lines[-1] = lines[-1][:-2] + "\n"
    
    lines.append("]\n")
    
    return ''.join(lines)

def main():
    """Main function to discover and report models."""
    print("=" * 60)
    print("Model Discovery Report")
    print("=" * 60)
    
    result = export_model_list()
    
    print("\nModels by category:")
    for category, models in result['categorized'].items():
        print(f"\n{category.replace('_', ' ').title()}: {len(models)} models")
        for model in models[:5]:  # Show first 5
            print(f"  - {model}")
        if len(models) > 5:
            print(f"  ... and {len(models) - 5} more")
    
    # Generate Python code
    python_code = generate_python_list()
    
    # Save to a file that can be imported
    code_file = Path(__file__).parent / "model_list.py"
    with open(code_file, 'w') as f:
        f.write(python_code)
    
    print(f"\nPython model list saved to: {code_file}")
    print("\nYou can now import this in p_sim_score.py:")
    print("  from model_list import test_model_list")

if __name__ == "__main__":
    main()