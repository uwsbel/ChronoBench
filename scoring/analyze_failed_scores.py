#!/usr/bin/env python3
"""
Analyze score extraction failures and provide a report without modifying files.
"""

import re
import os
import ast
import operator
import csv
from pathlib import Path
from collections import defaultdict

class MathEvaluator:
    """Safely evaluate simple mathematical expressions"""
    
    ops = {
        ast.Add: operator.add,
        ast.Sub: operator.sub,
        ast.Mult: operator.mul,
        ast.Div: operator.truediv,
        ast.USub: operator.neg,
    }
    
    def eval_expr(self, expr):
        """Safely evaluate a mathematical expression string."""
        try:
            node = ast.parse(expr, mode='eval').body
            return int(self._eval(node))
        except:
            return None
    
    def _eval(self, node):
        if isinstance(node, ast.Num):
            return node.n
        elif isinstance(node, ast.BinOp):
            return self.ops[type(node.op)](
                self._eval(node.left), 
                self._eval(node.right)
            )
        elif isinstance(node, ast.UnaryOp):
            return self.ops[type(node.op)](self._eval(node.operand))
        else:
            raise TypeError(node)

def analyze_score_file(file_path):
    """
    Analyze a score file and extract the score using various patterns.
    Returns: (score, format_type, original_text)
    """
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
        
        # Check for error messages
        if content.startswith("Error:") or content.startswith("FAILED:"):
            return (None, "error", "File contains error")
        
        # Try standard format [[70]]
        match = re.search(r"\[\[(\d+)\]\]", content)
        if match:
            score = int(match.group(1))
            return (score, "standard", f"[[{score}]]")
        
        # Try alternate format [[x]] 70
        match_x = re.search(r"\[\[x\]\]\s*(\d+)", content)
        if match_x:
            score = int(match_x.group(1))
            return (score, "x_format", f"[[x]] {score}")
        
        # Try math expression format [[40 - 2]]
        match_expr = re.search(r"\[\[([0-9\s\+\-\*/\(\)]+)\]\]", content)
        if match_expr:
            expr = match_expr.group(1).strip()
            evaluator = MathEvaluator()
            score = evaluator.eval_expr(expr)
            if score is not None:
                return (score, "math_expr", f"[[{expr}]]")
            else:
                return (None, "math_failed", f"[[{expr}]]")
        
        return (None, "not_found", "No score pattern found")
        
    except Exception as e:
        return (None, "error", str(e))

def main():
    """Analyze all score files and generate a report."""
    
    output_path = "/home/hongyu/Documents/SimBench/output_llms_gpt-4-1-nano"
    
    # Statistics
    stats = defaultdict(int)
    failed_files = []
    math_expr_files = []
    
    # Analyze all score files
    for root, dirs, files in os.walk(output_path):
        for file in files:
            if file.endswith('_score_document.txt') or \
               file.endswith('_score_reference.txt') or \
               file.endswith('_score_reference_document.txt'):
                
                file_path = os.path.join(root, file)
                score, format_type, original = analyze_score_file(file_path)
                
                stats[format_type] += 1
                
                if format_type == "math_expr":
                    rel_path = os.path.relpath(file_path, output_path)
                    math_expr_files.append({
                        'path': rel_path,
                        'expression': original,
                        'score': score
                    })
                
                if score is None:
                    rel_path = os.path.relpath(file_path, output_path)
                    failed_files.append({
                        'path': rel_path,
                        'format': format_type,
                        'reason': original
                    })
    
    # Generate report
    print("\n" + "="*60)
    print("SCORE EXTRACTION ANALYSIS REPORT")
    print("="*60)
    
    print("\n📊 Format Statistics:")
    print("-"*40)
    for format_type, count in sorted(stats.items()):
        print(f"  {format_type:15} : {count:5} files")
    
    print(f"\n  Total files    : {sum(stats.values()):5} files")
    
    print("\n🔢 Math Expression Files:")
    print("-"*40)
    if math_expr_files:
        print(f"Found {len(math_expr_files)} files with math expressions:\n")
        for item in math_expr_files[:10]:  # Show first 10
            print(f"  {item['path']}")
            print(f"    Expression: {item['expression']} = {item['score']}")
        if len(math_expr_files) > 10:
            print(f"\n  ... and {len(math_expr_files) - 10} more files")
    else:
        print("  No math expression files found")
    
    print("\n❌ Failed Extractions:")
    print("-"*40)
    if failed_files:
        print(f"Found {len(failed_files)} files with extraction failures:\n")
        
        # Group by failure type
        by_type = defaultdict(list)
        for item in failed_files:
            by_type[item['format']].append(item)
        
        for format_type, items in by_type.items():
            print(f"\n  {format_type} ({len(items)} files):")
            for item in items[:3]:  # Show first 3 of each type
                print(f"    {item['path']}")
            if len(items) > 3:
                print(f"    ... and {len(items) - 3} more")
    else:
        print("  No extraction failures found")
    
    # Save detailed report to CSV
    report_path = "/home/hongyu/Documents/SimBench/scoring/score_extraction_report.csv"
    with open(report_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['File Path', 'Format Type', 'Score', 'Original Text'])
        
        for root, dirs, files in os.walk(output_path):
            for file in files:
                if file.endswith('_score_document.txt') or \
                   file.endswith('_score_reference.txt') or \
                   file.endswith('_score_reference_document.txt'):
                    
                    file_path = os.path.join(root, file)
                    score, format_type, original = analyze_score_file(file_path)
                    rel_path = os.path.relpath(file_path, output_path)
                    writer.writerow([rel_path, format_type, score if score else 'None', original])
    
    print(f"\n📄 Detailed report saved to: {report_path}")
    
    # Suggest fix command
    if math_expr_files or failed_files:
        print("\n💡 To fix these issues, run:")
        print("   python /home/hongyu/Documents/SimBench/scoring/fix_math_expression_scores.py")

if __name__ == "__main__":
    main()