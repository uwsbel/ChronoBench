#!/usr/bin/env python3
"""
Fix score extraction for files with mathematical expressions in brackets.
This script specifically targets files where scores are in format [[40 - 2]] 
and extracts the correct score by evaluating the expression.
"""

import re
import os
import ast
import operator
import logging
from pathlib import Path

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# Safe math evaluation
class MathEvaluator:
    """Safely evaluate simple mathematical expressions"""
    
    # Supported operators
    ops = {
        ast.Add: operator.add,
        ast.Sub: operator.sub,
        ast.Mult: operator.mul,
        ast.Div: operator.truediv,
        ast.Pow: operator.pow,
        ast.USub: operator.neg,
    }
    
    def eval_expr(self, expr):
        """
        Safely evaluate a mathematical expression string.
        Only supports basic arithmetic operations.
        """
        try:
            node = ast.parse(expr, mode='eval').body
            return int(self._eval(node))
        except:
            return None
    
    def _eval(self, node):
        if isinstance(node, ast.Num):  # <number>
            return node.n
        elif isinstance(node, ast.BinOp):  # <left> <operator> <right>
            return self.ops[type(node.op)](
                self._eval(node.left), 
                self._eval(node.right)
            )
        elif isinstance(node, ast.UnaryOp):  # <operator> <operand>
            return self.ops[type(node.op)](self._eval(node.operand))
        else:
            raise TypeError(node)

def extract_score_from_file(file_path):
    """
    Extract score from a file, handling multiple formats:
    1. [[70]] - Standard format
    2. [[x]] 70 - Alternate format  
    3. [[40 - 2]] - Math expression format
    """
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
        
        # Check for error messages
        if content.startswith("Error:") or content.startswith("FAILED:"):
            logger.warning(f"File contains error: {file_path}")
            return None
        
        # Try standard format [[70]]
        match = re.search(r"\[\[(\d+)\]\]", content)
        if match:
            score = int(match.group(1))
            logger.debug(f"Found standard format score {score} in {file_path}")
            return score
        
        # Try alternate format [[x]] 70
        match_x = re.search(r"\[\[x\]\]\s*(\d+)", content)
        if match_x:
            score = int(match_x.group(1))
            logger.debug(f"Found [[x]] format score {score} in {file_path}")
            return score
        
        # Try math expression format [[40 - 2]]
        match_expr = re.search(r"\[\[([0-9\s\+\-\*/\(\)]+)\]\]", content)
        if match_expr:
            expr = match_expr.group(1).strip()
            evaluator = MathEvaluator()
            score = evaluator.eval_expr(expr)
            if score is not None:
                logger.info(f"Evaluated expression '{expr}' = {score} in {file_path}")
                return score
            else:
                logger.warning(f"Failed to evaluate expression '{expr}' in {file_path}")
        
        logger.warning(f"No valid score found in {file_path}")
        return None
        
    except Exception as e:
        logger.error(f"Error processing {file_path}: {e}")
        return None

def fix_score_in_file(file_path):
    """
    Fix the score in a file by extracting it properly and updating the file.
    This function reads the score, then rewrites the last part of the file 
    with the correct score format.
    """
    score = extract_score_from_file(file_path)
    
    if score is None:
        logger.warning(f"Could not extract score from {file_path}")
        return False
    
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
        
        # Find and replace the math expression with the evaluated score
        # Look for patterns like [[40 - 2]] and replace with [[38]]
        pattern = r"\[\[([0-9\s\+\-\*/\(\)]+)\]\]"
        
        def replace_expr(match):
            expr = match.group(1).strip()
            evaluator = MathEvaluator()
            result = evaluator.eval_expr(expr)
            if result is not None:
                return f"[[{result}]]"
            return match.group(0)  # Return original if can't evaluate
        
        new_content = re.sub(pattern, replace_expr, content)
        
        if new_content != content:
            # Write the updated content back
            with open(file_path, 'w', encoding='utf-8') as f:
                f.write(new_content)
            logger.info(f"✅ Fixed score in {file_path}")
            return True
        else:
            logger.debug(f"No changes needed for {file_path}")
            return False
            
    except Exception as e:
        logger.error(f"Error fixing {file_path}: {e}")
        return False

def main():
    """
    Main function to fix all files with math expression scores.
    """
    
    # Read list of files to fix
    files_list_path = "/tmp/files_to_fix.txt"
    
    if not os.path.exists(files_list_path):
        # If list doesn't exist, find files ourselves
        logger.info("Files list not found, searching for files with math expressions...")
        import subprocess
        result = subprocess.run(
            ['grep', '-r', r'\[\[[0-9].*[-+*/].*[0-9]\]\]', 
             '/home/hongyu/Documents/SimBench/output_llms_gpt-4-1-nano/',
             '--include=*.txt'],
            capture_output=True, text=True
        )
        
        files_to_fix = []
        for line in result.stdout.splitlines():
            if ':' in line:
                file_path = line.split(':', 1)[0]
                if file_path not in files_to_fix:
                    files_to_fix.append(file_path)
    else:
        # Read the pre-generated list
        with open(files_list_path, 'r') as f:
            files_to_fix = [line.strip() for line in f if line.strip()]
    
    logger.info(f"Found {len(files_to_fix)} files to check")
    
    fixed_count = 0
    failed_count = 0
    
    for file_path in files_to_fix:
        if os.path.exists(file_path):
            if fix_score_in_file(file_path):
                fixed_count += 1
            else:
                failed_count += 1
        else:
            logger.warning(f"File not found: {file_path}")
            failed_count += 1
    
    logger.info(f"\n=== Summary ===")
    logger.info(f"✅ Fixed: {fixed_count} files")
    logger.info(f"❌ Failed: {failed_count} files")
    logger.info(f"📊 Total: {len(files_to_fix)} files")
    
    # Now regenerate the CSV files for affected models
    if fixed_count > 0:
        logger.info("\n=== Regenerating CSV files ===")
        regenerate_csvs()

def regenerate_csvs():
    """
    Regenerate CSV files for models that had fixes applied.
    """
    import csv
    from collections import defaultdict
    
    output_path = "/home/hongyu/Documents/SimBench/output_llms_gpt-4-1-nano"
    
    # Find which models were affected
    affected_models = set()
    for root, dirs, files in os.walk(output_path):
        for file in files:
            if file == "evaluation_scores.csv":
                model_name = os.path.basename(os.path.dirname(root))
                affected_models.add(model_name)
    
    logger.info(f"Regenerating CSVs for {len(affected_models)} models")
    
    # For each affected model, regenerate the evaluation_scores.csv
    for model in affected_models:
        model_path = os.path.join(output_path, model)
        if not os.path.exists(model_path):
            continue
            
        # Collect all scores for this model
        all_scores = []
        
        for system_name in os.listdir(model_path):
            system_path = os.path.join(model_path, system_name)
            if not os.path.isdir(system_path):
                continue
            
            # Extract scores for each round
            for round_name in ['first', 'second', 'third']:
                row = [model, system_name, round_name]
                
                # Score types
                score_files = [
                    f"{round_name}_score_document.txt",
                    f"{round_name}_score_reference.txt", 
                    f"{round_name}_score_reference_document.txt"
                ]
                
                for score_file in score_files:
                    file_path = os.path.join(system_path, score_file)
                    if os.path.exists(file_path):
                        score = extract_score_from_file(file_path)
                        row.append(score if score is not None else 0)
                    else:
                        row.append(0)
                
                if len(row) == 6:  # model, system, round, 3 scores
                    all_scores.append(row)
        
        # Write the CSV for this model
        if all_scores:
            csv_path = os.path.join(model_path, "evaluation_scores_fixed.csv")
            with open(csv_path, 'w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow(["Test Model", "System", "Round", 
                               "Score Document", "Score Reference", 
                               "Score Reference Document"])
                writer.writerows(all_scores)
            logger.info(f"  ✅ Regenerated CSV for {model}")

if __name__ == "__main__":
    main()