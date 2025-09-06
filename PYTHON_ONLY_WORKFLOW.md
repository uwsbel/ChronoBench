# SimBench Python-Only Complete Workflow Guide

A robust, error-free pipeline using only Python scripts for the 3 JLLMs evaluation process with comprehensive error handling, state management, and recovery mechanisms.

## Table of Contents
1. [Prerequisites](#prerequisites)
2. [Pre-flight Checks](#pre-flight-checks)
3. [Step-by-Step Pipeline](#step-by-step-pipeline)
4. [Complete Automated Pipeline](#complete-automated-pipeline)
5. [Monitoring and Validation](#monitoring-and-validation)
6. [Troubleshooting](#troubleshooting)
7. [Resume from Failure](#resume-from-failure)

## ⚠️ CRITICAL ISSUES TO FIX FIRST

### Known Problems in Original Scripts
1. **extractPy.py is hardcoded to only process ONE model** (`gpt-4o-mini` on line 127)
   - This causes `p_sim_score.py` to fail when it tries to evaluate ALL models
   - You MUST extract all models before running similarity scoring

2. **Missing extraction validation**
   - No checks between extraction and scoring steps
   - Scripts assume all files exist but don't verify

3. **Order dependencies**
   - Must run extraction for ALL models before ANY scoring
   - Must validate extraction completeness before proceeding

### Quick Fix
```python
# Run this FIRST to extract all models:
cd scoring
python extract_all_models.py

# Validate extraction is complete:
python validate_extraction.py

# Only then proceed with similarity scoring:
python p_sim_score.py
```

## Prerequisites

### Required Packages
```python
# Install required packages
pip install openai anthropic google-generativeai mistralai
pip install pandas numpy scipy scikit-learn
pip install rouge-score nltk codebleu
pip install timeout-decorator tqdm
pip install python-dotenv
```

### Environment Setup
Create a `.env` file in the repository root:
```bash
# Primary API Keys
OPENAI_API_KEY="sk-..."
ANTHROPIC_API_KEY="..."
GOOGLE_API_KEY="..."
MISTRAL_API_KEY="..."
NVIDIA_API_KEY="..."

# Separate keys for parallel JLLM execution (to avoid rate limits)
OPENAI_API_KEY_1="sk-..."  # For gpt-4o-mini
OPENAI_API_KEY_2="sk-..."  # For gpt-4.1-mini
OPENAI_API_KEY_3="sk-..."  # For gpt-4.1-nano
```

## Pre-flight Checks

Run this script first to check what already exists and what needs to be done:

```python
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
```

## Step-by-Step Pipeline

### Step 0: Check Current State
```python
# Always run this first to see what's already done
python preflight_check.py
```

### Step 1: Clean Ground Truth Data (if needed)
```python
cd scoring
python clean_truth.py
```

### Step 2: Extract Python Code (if simulations exist but not extracted)

**IMPORTANT**: The original `extractPy.py` is hardcoded to only process `gpt-4o-mini`. You must extract ALL models before running similarity scores.

**Option A: Use the new extraction script (Recommended)**
```python
cd scoring
python extract_all_models.py
```

**Option B: Fix and use original extractPy.py**
```python
# Edit extractPy.py line 127 to include all models:
# test_model_list = ["gpt-4o-mini"]  # Original - only one model!
# Change to:
# test_model_list = [d.name for d in Path(Output_path).iterdir() if d.is_dir()]

cd scoring
python extractPy.py
```

**Validate extraction is complete:**
```python
python validate_extraction.py
```

### Step 3: Evaluate Code Execution
```python
cd scoring
python evaluatePy.py
```

### Step 4: Calculate Similarity Scores
```python
cd scoring
python p_sim_score.py
python p_sim_score_simple.py
python p_NIM_PE.py
```

### Step 5: Run 3 JLLMs Evaluation (with error handling)

```python
# robust_jllm_runner.py
import os
import sys
import subprocess
import json
import time
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor, as_completed
from datetime import datetime
import logging

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('jllm_runner.log'),
        logging.StreamHandler()
    ]
)

class JLLMRunner:
    def __init__(self):
        self.base_dir = Path.cwd()
        self.scoring_dir = self.base_dir / 'scoring'
        self.checkpoint_file = self.base_dir / '.jllm_checkpoint.json'
        self.load_checkpoint()
    
    def load_checkpoint(self):
        """Load checkpoint to resume from failure"""
        if self.checkpoint_file.exists():
            with open(self.checkpoint_file, 'r') as f:
                self.checkpoint = json.load(f)
        else:
            self.checkpoint = {
                'completed': [],
                'failed': [],
                'in_progress': []
            }
    
    def save_checkpoint(self):
        """Save current progress"""
        with open(self.checkpoint_file, 'w') as f:
            json.dump(self.checkpoint, f, indent=2)
    
    def run_single_jllm(self, model_info):
        """Run a single JLLM with retry logic"""
        model_name, script, api_key_var = model_info
        
        # Skip if already completed
        if model_name in self.checkpoint['completed']:
            logging.info(f"Skipping {model_name} - already completed")
            return model_name, True
        
        max_retries = 3
        retry_delay = 60  # seconds
        
        for attempt in range(max_retries):
            try:
                logging.info(f"Running {model_name} (attempt {attempt + 1}/{max_retries})")
                
                env = os.environ.copy()
                env['OPENAI_API_KEY'] = os.getenv(api_key_var, os.getenv('OPENAI_API_KEY'))
                
                if not env['OPENAI_API_KEY']:
                    logging.error(f"No API key found for {api_key_var}")
                    return model_name, False
                
                # Mark as in progress
                self.checkpoint['in_progress'].append(model_name)
                self.save_checkpoint()
                
                # Run the script
                script_path = self.scoring_dir / 'v01' / script
                result = subprocess.run(
                    [sys.executable, str(script_path)],
                    capture_output=True,
                    text=True,
                    env=env,
                    timeout=7200  # 2 hour timeout
                )
                
                if result.returncode == 0:
                    logging.info(f"✓ {model_name} completed successfully")
                    self.checkpoint['completed'].append(model_name)
                    if model_name in self.checkpoint['in_progress']:
                        self.checkpoint['in_progress'].remove(model_name)
                    self.save_checkpoint()
                    return model_name, True
                else:
                    logging.warning(f"✗ {model_name} failed with code {result.returncode}")
                    if attempt < max_retries - 1:
                        logging.info(f"Retrying in {retry_delay} seconds...")
                        time.sleep(retry_delay)
                        retry_delay *= 2  # Exponential backoff
                    
            except subprocess.TimeoutExpired:
                logging.error(f"✗ {model_name} timed out")
            except Exception as e:
                logging.error(f"✗ {model_name} error: {str(e)}")
            
        # Mark as failed after all retries
        self.checkpoint['failed'].append(model_name)
        if model_name in self.checkpoint['in_progress']:
            self.checkpoint['in_progress'].remove(model_name)
        self.save_checkpoint()
        return model_name, False
    
    def run_parallel(self):
        """Run JLLMs in parallel with error recovery"""
        jllm_configs = [
            ('gpt-4o-mini', 'p_JLLM_score_gpt4omini.py', 'OPENAI_API_KEY_1'),
            ('gpt-4.1-mini', 'p_JLLM_score_gpt41mini.py', 'OPENAI_API_KEY_2'),
            ('gpt-4.1-nano', 'p_JLLM_score_gpt41nano.py', 'OPENAI_API_KEY_3')
        ]
        
        # Filter out already completed
        to_run = [c for c in jllm_configs if c[0] not in self.checkpoint['completed']]
        
        if not to_run:
            logging.info("All JLLMs already completed!")
            return True
        
        logging.info(f"Running {len(to_run)} JLLM evaluations...")
        
        with ProcessPoolExecutor(max_workers=3) as executor:
            futures = {executor.submit(self.run_single_jllm, config): config[0] 
                      for config in to_run}
            
            for future in as_completed(futures):
                model_name = futures[future]
                try:
                    name, success = future.result()
                    if not success:
                        logging.error(f"Failed to complete {name}")
                except Exception as e:
                    logging.error(f"Exception for {model_name}: {str(e)}")
        
        # Check if all completed
        all_success = len(self.checkpoint['failed']) == 0
        return all_success
    
    def run_sequential(self):
        """Fallback to sequential execution if parallel fails"""
        logging.info("Running JLLMs sequentially...")
        
        jllm_configs = [
            ('gpt-4o-mini', 'p_JLLM_score_gpt4omini.py', 'OPENAI_API_KEY_1'),
            ('gpt-4.1-mini', 'p_JLLM_score_gpt41mini.py', 'OPENAI_API_KEY_2'),
            ('gpt-4.1-nano', 'p_JLLM_score_gpt41nano.py', 'OPENAI_API_KEY_3')
        ]
        
        for config in jllm_configs:
            self.run_single_jllm(config)
            time.sleep(10)  # Brief pause between models

if __name__ == "__main__":
    from dotenv import load_dotenv
    load_dotenv()
    
    runner = JLLMRunner()
    success = runner.run_parallel()
    
    if not success:
        print("\nSome JLLMs failed. Check jllm_runner.log for details.")
        print("You can re-run this script to retry failed evaluations.")
```

### Step 6: Generate Rankings
```python
cd scoring

# Merge evaluations
python merge_jllm_evaluations.py

# Generate rankings
python generate_rankings_for_judges.py
python generate_jllm_all_scores_ranked.py
python create_final_combined_scores.py
python generate_full_rankings.py
```

### Step 7: Analysis and Validation
```python
cd scoring

# Analyze and fix issues
python analyze_failed_scores.py
python check_consistency.py
python fix_error_files.py
python fix_math_expression_scores.py
python compute_missing_pe_metrics.py
python update_all_jllm_metrics.py
python sync_evaluation_results.py
```

## Complete Automated Pipeline

```python
# run_complete_pipeline.py
#!/usr/bin/env python3
"""
Complete SimBench evaluation pipeline with error handling and state management
"""

import os
import sys
import subprocess
import json
import time
import logging
from pathlib import Path
from datetime import datetime
from concurrent.futures import ProcessPoolExecutor, as_completed

# Configure logging
log_file = f"pipeline_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler(log_file),
        logging.StreamHandler()
    ]
)

class SimBenchPipeline:
    def __init__(self):
        self.base_dir = Path.cwd()
        self.scoring_dir = self.base_dir / 'scoring'
        self.state_file = self.base_dir / '.pipeline_state.json'
        self.checkpoint_file = self.base_dir / '.pipeline_checkpoint.json'
        self.load_state()
        self.load_checkpoint()
    
    def load_state(self):
        """Load pipeline state"""
        if self.state_file.exists():
            with open(self.state_file, 'r') as f:
                self.state = json.load(f)
        else:
            self.state = self.check_current_state()
    
    def load_checkpoint(self):
        """Load checkpoint for resuming"""
        if self.checkpoint_file.exists():
            with open(self.checkpoint_file, 'r') as f:
                self.checkpoint = json.load(f)
        else:
            self.checkpoint = {
                'completed_steps': [],
                'failed_steps': [],
                'last_run': None
            }
    
    def save_checkpoint(self):
        """Save current progress"""
        self.checkpoint['last_run'] = datetime.now().isoformat()
        with open(self.checkpoint_file, 'w') as f:
            json.dump(self.checkpoint, f, indent=2)
    
    def check_current_state(self):
        """Check what already exists"""
        state = {
            'simulations_exist': False,
            'extractions_exist': False,
            'evaluations_exist': False,
            'jllm_complete': {},
            'rankings_exist': False
        }
        
        # Check simulations
        output_llms = self.base_dir / 'output_llms'
        if output_llms.exists():
            models = [d for d in output_llms.iterdir() if d.is_dir()]
            state['simulations_exist'] = len(models) > 0
            
            # Check extractions
            cleaned = list(output_llms.rglob("*_cleaned_response.py"))
            state['extractions_exist'] = len(cleaned) > 0
        
        # Check JLLM evaluations
        for model in ['gpt-4o-mini', 'gpt-4-1-mini', 'gpt-4-1-nano']:
            model_dir = self.scoring_dir / 'out_diff_models' / f"out_{model.replace('.', '-')}"
            state['jllm_complete'][model] = (model_dir / 'evaluation_results.csv').exists()
        
        # Check rankings
        rankings_file = self.scoring_dir / 'out' / 'all_scores_ranked.csv'
        state['rankings_exist'] = rankings_file.exists()
        
        return state
    
    def run_step(self, step_name, script_path, working_dir=None, skip_if_exists=None):
        """Run a single pipeline step with error handling"""
        
        # Check if step should be skipped
        if skip_if_exists and skip_if_exists():
            logging.info(f"Skipping {step_name} - already completed")
            return True
        
        if step_name in self.checkpoint['completed_steps']:
            logging.info(f"Skipping {step_name} - found in checkpoint")
            return True
        
        logging.info(f"Running {step_name}...")
        
        try:
            cwd = working_dir or self.base_dir
            result = subprocess.run(
                [sys.executable, script_path],
                capture_output=True,
                text=True,
                cwd=str(cwd),
                timeout=3600  # 1 hour timeout
            )
            
            if result.returncode == 0:
                logging.info(f"✓ {step_name} completed successfully")
                self.checkpoint['completed_steps'].append(step_name)
                self.save_checkpoint()
                return True
            else:
                logging.error(f"✗ {step_name} failed with code {result.returncode}")
                logging.error(f"Error output: {result.stderr[:500]}")
                self.checkpoint['failed_steps'].append(step_name)
                self.save_checkpoint()
                return False
                
        except subprocess.TimeoutExpired:
            logging.error(f"✗ {step_name} timed out")
            self.checkpoint['failed_steps'].append(step_name)
            self.save_checkpoint()
            return False
        except Exception as e:
            logging.error(f"✗ {step_name} error: {str(e)}")
            self.checkpoint['failed_steps'].append(step_name)
            self.save_checkpoint()
            return False
    
    def run_jllms(self):
        """Run JLLM evaluations with proper error handling"""
        # Check which JLLMs need to run
        to_run = [m for m, done in self.state['jllm_complete'].items() if not done]
        
        if not to_run:
            logging.info("All JLLM evaluations already complete")
            return True
        
        logging.info(f"Running JLLM evaluations for: {', '.join(to_run)}")
        
        # Import and run the robust JLLM runner
        try:
            from robust_jllm_runner import JLLMRunner
            runner = JLLMRunner()
            return runner.run_parallel()
        except ImportError:
            logging.warning("Robust JLLM runner not found, using basic approach")
            
            # Fallback to basic sequential execution
            scripts = {
                'gpt-4o-mini': 'p_JLLM_score_gpt4omini.py',
                'gpt-4.1-mini': 'p_JLLM_score_gpt41mini.py',
                'gpt-4.1-nano': 'p_JLLM_score_gpt41nano.py'
            }
            
            success = True
            for model in to_run:
                if model in scripts:
                    script_path = self.scoring_dir / 'v01' / scripts[model]
                    if not self.run_step(f"JLLM_{model}", script_path, self.scoring_dir):
                        success = False
            
            return success
    
    def validate_results(self):
        """Validate that all expected outputs exist"""
        validation = {
            'all_pass': True,
            'checks': {}
        }
        
        # Check JLLM outputs
        for model in ['gpt-4o-mini', 'gpt-4-1-mini', 'gpt-4-1-nano']:
            model_dir = self.scoring_dir / 'out_diff_models' / f"out_{model.replace('.', '-')}"
            expected_files = [
                'evaluation_results.csv',
                'combined_evaluation_scores_*.csv',
                'all_scores_ranked.csv'
            ]
            
            model_valid = True
            for pattern in expected_files:
                if '*' in pattern:
                    files = list(model_dir.glob(pattern))
                    if not files:
                        model_valid = False
                        logging.warning(f"Missing {pattern} for {model}")
                else:
                    if not (model_dir / pattern).exists():
                        model_valid = False
                        logging.warning(f"Missing {pattern} for {model}")
            
            validation['checks'][model] = model_valid
            if not model_valid:
                validation['all_pass'] = False
        
        # Check final rankings
        final_files = [
            self.scoring_dir / 'out' / 'all_scores_ranked.csv',
            self.scoring_dir / 'out' / 'final_combined_scores.csv'
        ]
        
        for f in final_files:
            if not f.exists():
                validation['all_pass'] = False
                logging.warning(f"Missing final output: {f.name}")
        
        return validation
    
    def run(self):
        """Run the complete pipeline"""
        print("="*60)
        print("SimBench Complete Evaluation Pipeline")
        print("="*60)
        
        # Check current state
        self.state = self.check_current_state()
        
        # Step 1: Check if simulations exist
        if not self.state['simulations_exist']:
            logging.error("No simulation outputs found in output_llms/")
            logging.info("Please generate simulations first or check the path")
            return False
        
        # Step 2: Clean ground truth (always safe to run)
        self.run_step(
            "Clean Ground Truth",
            self.scoring_dir / "clean_truth.py",
            self.scoring_dir
        )
        
        # Step 3: Extract Python code for ALL models
        if not self.state['extractions_exist']:
            # Use the new extraction script that processes ALL models
            success = self.run_step(
                "Extract Python Code (ALL models)",
                self.scoring_dir / "extract_all_models.py",
                self.scoring_dir
            )
            if not success:
                logging.error("Extraction failed - cannot continue")
                return False
            
            # Validate extraction completeness
            validation = self.run_step(
                "Validate Extraction",
                self.scoring_dir / "validate_extraction.py",
                self.scoring_dir
            )
            if not validation:
                logging.warning("Some models may have incomplete extraction")
        
        # Step 4: Evaluate code execution
        self.run_step(
            "Evaluate Code Execution",
            self.scoring_dir / "evaluatePy.py",
            self.scoring_dir
        )
        
        # Step 5: Calculate similarity scores
        for script in ['p_sim_score.py', 'p_sim_score_simple.py', 'p_NIM_PE.py']:
            self.run_step(
                f"Similarity Score ({script})",
                self.scoring_dir / script,
                self.scoring_dir
            )
        
        # Step 6: Run JLLM evaluations
        jllm_success = self.run_jllms()
        if not jllm_success:
            logging.warning("Some JLLM evaluations failed")
        
        # Step 7: Generate rankings
        ranking_scripts = [
            'merge_jllm_evaluations.py',
            'generate_rankings_for_judges.py',
            'generate_jllm_all_scores_ranked.py',
            'create_final_combined_scores.py',
            'generate_full_rankings.py'
        ]
        
        for script in ranking_scripts:
            self.run_step(
                f"Rankings ({script})",
                self.scoring_dir / script,
                self.scoring_dir
            )
        
        # Step 8: Analysis and fixes
        analysis_scripts = [
            'analyze_failed_scores.py',
            'check_consistency.py',
            'fix_error_files.py',
            'fix_math_expression_scores.py',
            'compute_missing_pe_metrics.py',
            'update_all_jllm_metrics.py',
            'sync_evaluation_results.py'
        ]
        
        for script in analysis_scripts:
            script_path = self.scoring_dir / script
            if script_path.exists():
                self.run_step(
                    f"Analysis ({script})",
                    script_path,
                    self.scoring_dir
                )
        
        # Final validation
        print("\n" + "="*60)
        print("VALIDATION RESULTS")
        print("="*60)
        
        validation = self.validate_results()
        if validation['all_pass']:
            print("✓ All validations passed!")
            print("\nResults available in:")
            print(f"  - {self.scoring_dir}/out_diff_models/")
            print(f"  - {self.scoring_dir}/out/")
        else:
            print("✗ Some validations failed - check logs for details")
            
        print(f"\nFull log available at: {log_file}")
        
        return validation['all_pass']

if __name__ == "__main__":
    from dotenv import load_dotenv
    load_dotenv()
    
    pipeline = SimBenchPipeline()
    success = pipeline.run()
    
    sys.exit(0 if success else 1)
```

## Monitoring and Validation

### Real-time Progress Monitor
```python
# monitor_progress.py
import time
import json
from pathlib import Path
from datetime import datetime

def monitor_jllm_logs():
    """Monitor JLLM evaluation progress in real-time"""
    log_files = {
        'gpt-4o-mini': 'scoring/out_diff_models/out_gpt-4o-mini/jllm_score_log.txt',
        'gpt-4.1-mini': 'scoring/out_diff_models/out_gpt-4-1-mini/jllm_score_log.txt',
        'gpt-4.1-nano': 'scoring/out_diff_models/out_gpt-4-1-nano/jllm_score_log.txt'
    }
    
    last_lines = {}
    
    print("Monitoring JLLM progress... (Ctrl+C to stop)")
    print("="*60)
    
    while True:
        for model, log_path in log_files.items():
            log_file = Path(log_path)
            if log_file.exists():
                with open(log_file, 'r') as f:
                    lines = f.readlines()
                    if lines:
                        current_last = lines[-1].strip()
                        if model not in last_lines or last_lines[model] != current_last:
                            last_lines[model] = current_last
                            
                            # Count progress
                            completed = sum(1 for line in lines if '✓ Completed:' in line)
                            failed = sum(1 for line in lines if '✗ Failed:' in line)
                            
                            print(f"\n[{datetime.now().strftime('%H:%M:%S')}] {model}:")
                            print(f"  Completed: {completed}, Failed: {failed}")
                            print(f"  Last: {current_last[:80]}")
        
        time.sleep(10)

if __name__ == "__main__":
    monitor_jllm_logs()
```

### Validation Script
```python
# validate_results.py
import pandas as pd
from pathlib import Path
import json

def validate_complete_pipeline():
    """Comprehensive validation of pipeline results"""
    
    results = {
        'timestamp': datetime.now().isoformat(),
        'checks': {},
        'summary': {}
    }
    
    base_dir = Path.cwd()
    scoring_dir = base_dir / 'scoring'
    
    # Check JLLM evaluations
    jllm_models = ['gpt-4o-mini', 'gpt-4-1-mini', 'gpt-4-1-nano']
    
    for model in jllm_models:
        model_dir = scoring_dir / 'out_diff_models' / f"out_{model.replace('.', '-')}"
        model_checks = {}
        
        # Check key files
        files_to_check = [
            'evaluation_results.csv',
            'all_scores_ranked.csv',
            'progress.json'
        ]
        
        for file_name in files_to_check:
            file_path = model_dir / file_name
            if file_path.exists():
                model_checks[file_name] = True
                
                # Additional validation for CSV files
                if file_name.endswith('.csv'):
                    try:
                        df = pd.read_csv(file_path)
                        model_checks[f"{file_name}_rows"] = len(df)
                    except Exception as e:
                        model_checks[f"{file_name}_error"] = str(e)
            else:
                model_checks[file_name] = False
        
        results['checks'][model] = model_checks
    
    # Check final outputs
    final_outputs = [
        scoring_dir / 'out' / 'all_scores_ranked.csv',
        scoring_dir / 'out' / 'final_combined_scores.csv'
    ]
    
    for output in final_outputs:
        if output.exists():
            results['checks'][output.name] = True
            try:
                df = pd.read_csv(output)
                results['summary'][output.name] = {
                    'exists': True,
                    'rows': len(df),
                    'columns': list(df.columns)
                }
            except Exception as e:
                results['summary'][output.name] = {'error': str(e)}
        else:
            results['checks'][output.name] = False
    
    # Save validation report
    report_file = base_dir / f"validation_report_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
    with open(report_file, 'w') as f:
        json.dump(results, f, indent=2)
    
    print(f"Validation report saved to: {report_file}")
    
    # Print summary
    print("\nValidation Summary:")
    print("="*60)
    
    all_pass = True
    for check, status in results['checks'].items():
        if isinstance(status, dict):
            check_pass = all(v for v in status.values() if isinstance(v, bool))
        else:
            check_pass = status
        
        symbol = "✓" if check_pass else "✗"
        print(f"{symbol} {check}")
        
        if not check_pass:
            all_pass = False
    
    print("="*60)
    print(f"Overall: {'PASS' if all_pass else 'FAIL'}")
    
    return all_pass

if __name__ == "__main__":
    from datetime import datetime
    validate_complete_pipeline()
```

## Troubleshooting

### Common Issues and Solutions

#### 1. API Rate Limits
```python
# If you encounter rate limits, use this helper:
import time

def wait_for_rate_limit(seconds=60):
    """Wait for rate limit to reset"""
    print(f"Rate limit hit. Waiting {seconds} seconds...")
    for i in range(seconds, 0, -10):
        print(f"  {i} seconds remaining...")
        time.sleep(10)
```

#### 2. Memory Issues
```python
# For large datasets, process in batches:
def process_in_batches(items, batch_size=100):
    """Process items in batches to avoid memory issues"""
    for i in range(0, len(items), batch_size):
        batch = items[i:i+batch_size]
        yield batch
```

#### 3. Corrupted Output Files
```python
# Check and clean corrupted CSV files:
def validate_csv(file_path):
    """Validate and repair CSV files"""
    try:
        df = pd.read_csv(file_path)
        print(f"✓ {file_path} is valid")
        return True
    except Exception as e:
        print(f"✗ {file_path} is corrupted: {e}")
        
        # Try to repair
        try:
            df = pd.read_csv(file_path, error_bad_lines=False)
            backup = file_path.replace('.csv', '_backup.csv')
            os.rename(file_path, backup)
            df.to_csv(file_path, index=False)
            print(f"  Repaired and saved. Backup at {backup}")
            return True
        except:
            return False
```

## Resume from Failure

To resume the pipeline after a failure:

```python
# resume_pipeline.py
import json
from pathlib import Path

def resume_pipeline():
    """Resume pipeline from last checkpoint"""
    checkpoint_file = Path('.pipeline_checkpoint.json')
    
    if not checkpoint_file.exists():
        print("No checkpoint found. Starting fresh...")
        from run_complete_pipeline import SimBenchPipeline
        pipeline = SimBenchPipeline()
        return pipeline.run()
    
    with open(checkpoint_file, 'r') as f:
        checkpoint = json.load(f)
    
    print(f"Resuming from checkpoint (last run: {checkpoint.get('last_run', 'unknown')})")
    print(f"Completed steps: {len(checkpoint.get('completed_steps', []))}")
    print(f"Failed steps: {len(checkpoint.get('failed_steps', []))}")
    
    if checkpoint.get('failed_steps'):
        print("\nFailed steps to retry:")
        for step in checkpoint['failed_steps']:
            print(f"  - {step}")
    
    # Resume
    from run_complete_pipeline import SimBenchPipeline
    pipeline = SimBenchPipeline()
    return pipeline.run()

if __name__ == "__main__":
    from dotenv import load_dotenv
    load_dotenv()
    
    resume_pipeline()
```

## Quick Reference

### Minimal Commands (if everything exists)
```python
# Just regenerate rankings from existing evaluations
cd scoring
python generate_jllm_all_scores_ranked.py
python create_final_combined_scores.py
```

### Full Pipeline (from scratch)
```python
# 1. Check state
python preflight_check.py

# 2. Run complete pipeline
python run_complete_pipeline.py

# 3. Validate results
python validate_results.py
```

### Monitor Progress
```python
# In a separate terminal
python monitor_progress.py
```

### Resume After Failure
```python
python resume_pipeline.py
```

## Important Notes

1. **API Keys**: Each JLLM should use a separate API key to avoid rate limits
2. **Parallel Execution**: The 3 JLLMs run in parallel by default with fallback to sequential
3. **Checkpointing**: All progress is saved and can be resumed after failures
4. **Validation**: Always run validation after completion to ensure all outputs are correct
5. **Logging**: Detailed logs are saved with timestamps for debugging

## Directory Structure After Completion

```
SimBench/
├── .pipeline_state.json           # Current state tracking
├── .pipeline_checkpoint.json      # Resume checkpoint
├── .jllm_checkpoint.json          # JLLM specific checkpoint
├── pipeline_*.log                 # Detailed execution logs
├── jllm_runner.log                # JLLM execution log
├── validation_report_*.json       # Validation results
├── output_llms/                   # S-LLM outputs (input)
│   └── [model_name]/
│       └── [system]/
│           ├── *_response.py
│           └── *_cleaned_response.py
├── scoring/
│   ├── execution.log              # Code execution results
│   ├── extraction.log             # Extraction process log
│   ├── out_diff_models/          # JLLM evaluation results
│   │   ├── out_gpt-4o-mini/
│   │   ├── out_gpt-4-1-mini/
│   │   └── out_gpt-4-1-nano/
│   └── out/                      # Final rankings
│       ├── all_scores_ranked.csv
│       └── final_combined_scores.csv
└── demo_data/                    # Ground truth (cleaned)
```

This workflow provides a complete, error-free pipeline with comprehensive error handling, state management, and recovery mechanisms.