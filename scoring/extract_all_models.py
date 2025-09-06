#!/usr/bin/env python3
"""
Extract Python code from ALL models in output_llms directory
This script automatically discovers and processes all models
"""

import os
import json
import re
import logging
from pathlib import Path
from tqdm import tqdm
from datetime import datetime
import sys

# Set up logging
log_file = f'extraction_all_{datetime.now().strftime("%Y%m%d_%H%M%S")}.log'
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler(log_file),
        logging.StreamHandler()
    ]
)

def extract_python_code(txt_file_path, output_py_file):
    """Extract Python code from response file"""
    try:
        with open(txt_file_path, 'r', encoding='utf-8') as file:
            content = file.read()

        python_code = ""
        
        # Try to find python code blocks
        multiple_matches = re.findall(r'```python(.*?)```', content, re.DOTALL)
        if multiple_matches:
            python_code = "\n\n".join(match.strip() for match in multiple_matches)
        else:
            # Check for single match
            start_match = re.search(r'```python', content)
            if start_match:
                end_match = re.search(r'```', content[start_match.end():])
                if end_match:
                    python_code = content[start_match.end():start_match.end() + end_match.start()].strip()
                else:
                    # Only start tag found
                    python_code = content[start_match.end():].strip()
                    python_code += '\n# Note: End marker ``` was missing'
            else:
                # No python tags at all - assume entire content is code
                python_code = content.strip()

        # Save the extracted code
        with open(output_py_file, 'w', encoding='utf-8') as py_file:
            py_file.write(python_code)

        return True

    except FileNotFoundError:
        logging.error(f"File not found: {txt_file_path}")
        return False
    except Exception as e:
        logging.error(f"Error processing {txt_file_path}: {str(e)}")
        return False

def remove_comments_from_file(input_py_file, output_py_file):
    """Remove comments from Python file"""
    try:
        with open(input_py_file, 'r', encoding='utf-8') as file:
            lines = file.readlines()

        cleaned_lines = []
        in_multiline_comment = False
        
        for line in lines:
            # Handle multi-line comments
            if '"""' in line or "'''" in line:
                quote_type = '"""' if '"""' in line else "'''"
                parts = line.split(quote_type)
                if len(parts) >= 3:  # Comment starts and ends on same line
                    cleaned_lines.append(parts[0] + parts[2])
                elif len(parts) == 2:
                    if in_multiline_comment:
                        cleaned_lines.append(parts[1])
                        in_multiline_comment = False
                    else:
                        cleaned_lines.append(parts[0])
                        in_multiline_comment = True
            elif in_multiline_comment:
                continue
            else:
                # Remove single-line comments
                if '#' in line:
                    code_part = line.split('#')[0]
                    if code_part.strip():
                        cleaned_lines.append(code_part + '\n')
                else:
                    cleaned_lines.append(line)

        # Write cleaned code
        with open(output_py_file, 'w', encoding='utf-8') as file:
            file.writelines(cleaned_lines)
        
        return True
        
    except Exception as e:
        logging.error(f"Error removing comments from {input_py_file}: {str(e)}")
        return False

def process_model(model_dir, system_list):
    """Process all systems for a single model"""
    model_name = model_dir.name
    stats = {'extracted': 0, 'cleaned': 0, 'failed': 0, 'skipped': 0}
    
    for system in system_list:
        system_path = model_dir / system
        if not system_path.exists():
            logging.warning(f"System {system} not found for model {model_name}")
            stats['failed'] += 1
            continue
        
        # Process three response files
        for response_num in ['first', 'second', 'third']:
            response_file = system_path / f"{response_num}_response.py"
            extracted_file = system_path / f"{response_num}_extracted.py"
            cleaned_file = system_path / f"{response_num}_cleaned_response.py"
            
            # Skip if cleaned file already exists
            if cleaned_file.exists():
                stats['skipped'] += 1
                continue
            
            if not response_file.exists():
                logging.warning(f"Missing {response_file}")
                stats['failed'] += 1
                continue
            
            # Extract Python code
            if extract_python_code(response_file, extracted_file):
                stats['extracted'] += 1
                
                # Remove comments
                if remove_comments_from_file(extracted_file, cleaned_file):
                    stats['cleaned'] += 1
                    # Remove intermediate file
                    extracted_file.unlink(missing_ok=True)
                else:
                    stats['failed'] += 1
            else:
                stats['failed'] += 1
    
    return stats

def main():
    """Main extraction function for all models"""
    base_dir = Path.cwd().parent  # Assuming script is in scoring/
    output_llms_dir = base_dir / 'output_llms'
    
    if not output_llms_dir.exists():
        logging.error(f"Directory not found: {output_llms_dir}")
        sys.exit(1)
    
    # System list from SimBench
    system_list = [
        "art", "beam", "buckling", "cable", "camera", "citybus", 
        "curiosity", "feda", "gator", "gear", "gps_imu", "handler", 
        "hmmwv", "kraz", "lidar", "m113", "man", "mass_spring_damper", 
        "particles", "pendulum", "rigid_highway", "rigid_multipatches", 
        "rotor", "scm", "scm_hill", "sedan", "sensros", "slider_crank",
        "tablecloth", "turtlebot", "uazbus", "veh_app", "vehros", "viper"
    ]
    
    # Get all model directories
    model_dirs = [d for d in output_llms_dir.iterdir() 
                  if d.is_dir() and not d.name.startswith('.')]
    
    if not model_dirs:
        logging.error("No model directories found")
        sys.exit(1)
    
    logging.info(f"Found {len(model_dirs)} models to process")
    logging.info(f"Processing {len(system_list)} systems per model")
    
    # Create checkpoint file for resuming
    checkpoint_file = base_dir / 'scoring' / '.extraction_checkpoint.json'
    checkpoint = {}
    
    if checkpoint_file.exists():
        with open(checkpoint_file, 'r') as f:
            checkpoint = json.load(f)
        logging.info(f"Resuming from checkpoint. Already processed: {len(checkpoint.get('completed', []))} models")
    else:
        checkpoint = {'completed': [], 'failed': [], 'stats': {}}
    
    # Process each model
    total_stats = {'extracted': 0, 'cleaned': 0, 'failed': 0, 'skipped': 0}
    
    for model_dir in tqdm(model_dirs, desc="Processing models"):
        model_name = model_dir.name
        
        # Skip if already processed
        if model_name in checkpoint.get('completed', []):
            logging.info(f"Skipping {model_name} - already completed")
            continue
        
        logging.info(f"Processing model: {model_name}")
        
        try:
            stats = process_model(model_dir, system_list)
            
            # Update totals
            for key in total_stats:
                total_stats[key] += stats[key]
            
            # Save to checkpoint
            checkpoint['completed'].append(model_name)
            checkpoint['stats'][model_name] = stats
            
            with open(checkpoint_file, 'w') as f:
                json.dump(checkpoint, f, indent=2)
            
            logging.info(f"Completed {model_name}: Extracted={stats['extracted']}, "
                        f"Cleaned={stats['cleaned']}, Failed={stats['failed']}, "
                        f"Skipped={stats['skipped']}")
            
        except Exception as e:
            logging.error(f"Failed to process {model_name}: {str(e)}")
            checkpoint['failed'].append(model_name)
            with open(checkpoint_file, 'w') as f:
                json.dump(checkpoint, f, indent=2)
    
    # Final summary
    print("\n" + "="*60)
    print("EXTRACTION COMPLETE")
    print("="*60)
    print(f"Total models processed: {len(checkpoint.get('completed', []))}")
    print(f"Total files extracted: {total_stats['extracted']}")
    print(f"Total files cleaned: {total_stats['cleaned']}")
    print(f"Total files skipped (already exist): {total_stats['skipped']}")
    print(f"Total failures: {total_stats['failed']}")
    
    if checkpoint.get('failed'):
        print(f"\nFailed models: {', '.join(checkpoint['failed'])}")
    
    print(f"\nLog file: {log_file}")
    print(f"Checkpoint file: {checkpoint_file}")
    
    # Verify completeness
    expected_files = len(model_dirs) * len(system_list) * 3  # 3 responses per system
    actual_cleaned = len(list(output_llms_dir.rglob("*_cleaned_response.py")))
    
    print(f"\nExpected cleaned files: {expected_files}")
    print(f"Actual cleaned files: {actual_cleaned}")
    
    if actual_cleaned < expected_files:
        print(f"WARNING: Missing {expected_files - actual_cleaned} cleaned files")
        print("Run 'python validate_extraction.py' to see which files are missing")
    else:
        print("✓ All files successfully extracted and cleaned!")
    
    return 0 if total_stats['failed'] == 0 else 1

if __name__ == "__main__":
    sys.exit(main())