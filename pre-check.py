# preflight_check.py
import os
import json
from pathlib import Path
from datetime import datetime
import sys

def check_environment():
    """Check if environment is properly configured"""
    checks = {
        'api_keys': False,
        'output_dirs': False,
        'simulations': False,
        'extractions': False,
        'evaluations': False,
        'jllm_evaluations': {}
    }
    
    # Check API keys
    api_keys = ['OPENAI_API_KEY_1', 'OPENAI_API_KEY_2', 'OPENAI_API_KEY_3']
    if all(os.getenv(key) for key in api_keys):
        checks['api_keys'] = True
        print("✓ API keys configured")
    else:
        missing = [key for key in api_keys if not os.getenv(key)]
        print(f"✗ Missing API keys: {', '.join(missing)}")
    
    # Check directories
    base_dir = Path.cwd()
    required_dirs = ['output_llms', 'scoring', 'demo_data']
    if all((base_dir / d).exists() for d in required_dirs):
        checks['output_dirs'] = True
        print("✓ Required directories exist")
    else:
        print("✗ Missing directories")
    
    # Check simulations
    output_llms = base_dir / 'output_llms'
    if output_llms.exists():
        models = [d for d in output_llms.iterdir() if d.is_dir()]
        if models:
            checks['simulations'] = True
            print(f"✓ Found {len(models)} model simulations")
            
            # Check extractions
            cleaned_files = list(output_llms.rglob("*_cleaned_response.py"))
            if cleaned_files:
                checks['extractions'] = True
                print(f"✓ Found {len(cleaned_files)} extracted Python files")
            else:
                print("✗ No extracted Python files found")
        else:
            print("✗ No simulation outputs found")
    
    # Check JLLM evaluations
    jllm_models = ['gpt-4o-mini', 'gpt-4-1-mini', 'gpt-4-1-nano']
    scoring_dir = base_dir / 'scoring' / 'out_diff_models'
    
    for model in jllm_models:
        model_dir = scoring_dir / f"out_{model.replace('.', '-')}"
        if model_dir.exists() and (model_dir / 'evaluation_results.csv').exists():
            checks['jllm_evaluations'][model] = True
            print(f"✓ {model} evaluation exists")
        else:
            checks['jllm_evaluations'][model] = False
            print(f"✗ {model} evaluation missing")
    
    # Save state
    state_file = base_dir / '.pipeline_state.json'
    with open(state_file, 'w') as f:
        json.dump(checks, f, indent=2)
    
    print(f"\nState saved to {state_file}")
    return checks

def recommend_actions(checks):
    """Recommend what steps to run based on current state"""
    print("\n" + "="*50)
    print("RECOMMENDED ACTIONS:")
    print("="*50)
    
    if not checks['api_keys']:
        print("1. Configure API keys in .env file")
        return
    
    if not checks['simulations']:
        print("1. Generate simulations (run S-LLM models)")
        print("   Note: This is time-consuming and requires API calls")
    elif not checks['extractions']:
        print("1. Run extraction: python scoring/extractPy.py")
    
    # Check which JLLMs need to run
    missing_jllms = [m for m, done in checks['jllm_evaluations'].items() if not done]
    if missing_jllms:
        print(f"2. Run JLLM evaluations for: {', '.join(missing_jllms)}")
    else:
        print("2. All JLLM evaluations complete - regenerate rankings only")
    
    if all(checks['jllm_evaluations'].values()):
        print("3. Generate final rankings and analysis")

if __name__ == "__main__":
    print("SimBench Pre-flight Check")
    print("="*50)
    
    # Load environment variables
    from dotenv import load_dotenv
    load_dotenv()
    
    checks = check_environment()
    recommend_actions(checks)