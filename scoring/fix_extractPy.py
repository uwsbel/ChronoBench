#!/usr/bin/env python3
"""
Fix extractPy.py to process ALL models instead of just gpt-4o-mini
"""

import os
from pathlib import Path

def fix_extract_script():
    """Update extractPy.py to process all models"""
    
    extract_script = Path("extractPy.py")
    
    if not extract_script.exists():
        print("Error: extractPy.py not found in current directory")
        return False
    
    # Read the script
    with open(extract_script, 'r') as f:
        lines = f.readlines()
    
    # Find and replace the test_model_list line
    modified = False
    for i, line in enumerate(lines):
        if line.strip().startswith('test_model_list = ["gpt-4o-mini"]'):
            print(f"Found hardcoded model list at line {i+1}")
            
            # Replace with dynamic model discovery
            new_lines = [
                '# FIXED: Now processes ALL models dynamically\n',
                'output_llms_path = Path(Output_path)\n',
                'test_model_list = [d.name for d in output_llms_path.iterdir() if d.is_dir() and not d.name.startswith(".")]\n',
                f'print(f"Found {{len(test_model_list)}} models to process")\n',
                f'print(f"Models: {{test_model_list[:5]}}..." if len(test_model_list) > 5 else f"Models: {{test_model_list}}")\n'
            ]
            
            # Replace the line
            lines[i] = ''.join(new_lines)
            modified = True
            break
    
    if not modified:
        print("Warning: Could not find the hardcoded model list")
        print("Looking for alternative patterns...")
        
        # Try to find any test_model_list assignment
        for i, line in enumerate(lines):
            if 'test_model_list' in line and '=' in line:
                print(f"Found test_model_list at line {i+1}: {line.strip()}")
    
    if modified:
        # Create backup
        backup_path = extract_script.with_suffix('.py.backup')
        with open(backup_path, 'w') as f:
            with open(extract_script, 'r') as original:
                f.write(original.read())
        print(f"Created backup: {backup_path}")
        
        # Write modified script
        with open(extract_script, 'w') as f:
            f.writelines(lines)
        
        print("✓ Successfully fixed extractPy.py to process ALL models")
        print("Run 'python extractPy.py' to extract all models")
        return True
    else:
        print("✗ Could not automatically fix extractPy.py")
        print("Please manually edit line 127 to include all models")
        return False

if __name__ == "__main__":
    import sys
    os.chdir(Path(__file__).parent)
    success = fix_extract_script()
    sys.exit(0 if success else 1)