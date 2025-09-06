#!/usr/bin/env python3
import json

for model in ["gpt-4-1-nano", "gpt-4-1-mini", "gpt-4o-mini"]:
    path = f"/home/hongyu/Documents/SimBench/scoring/out_diff_models/out_{model}/progress.json"
    
    with open(path, "r") as f:
        data = json.load(f)
    
    changes = []
    
    # Remove from completed
    if "completed" in data and isinstance(data["completed"], list):
        before = len(data["completed"])
        data["completed"] = [item for item in data["completed"] if not item.startswith("gemma-3-27b-it")]
        if before != len(data["completed"]):
            changes.append(f"Removed {before - len(data['completed'])} from completed")
    
    # Remove from in_progress  
    if "in_progress" in data:
        if isinstance(data["in_progress"], list):
            before = len(data["in_progress"])
            data["in_progress"] = [item for item in data["in_progress"] if item != "gemma-3-27b-it"]
            if before != len(data["in_progress"]):
                changes.append(f"Removed {before - len(data['in_progress'])} from in_progress")
    
    # Remove from errors section
    if "errors" in data and "gemma-3-27b-it" in data["errors"]:
        del data["errors"]["gemma-3-27b-it"]
        changes.append("Removed from errors section")
    
    # Remove from failed section
    if "failed" in data:
        if isinstance(data["failed"], dict) and "gemma-3-27b-it" in data["failed"]:
            del data["failed"]["gemma-3-27b-it"]
            changes.append("Removed from failed dict")
        elif isinstance(data["failed"], list):
            before = len(data["failed"])
            data["failed"] = [entry for entry in data["failed"]
                            if not (isinstance(entry, dict) and
                                  "key" in entry and
                                  entry["key"].startswith("gemma-3-27b-it/"))]
            if before != len(data["failed"]):
                changes.append(f"Removed {before - len(data['failed'])} from failed list")
    
    # Remove from error_log section
    if "error_log" in data and isinstance(data["error_log"], list):
        before = len(data["error_log"])
        data["error_log"] = [entry for entry in data["error_log"] 
                            if not (isinstance(entry, dict) and 
                                  "key" in entry and 
                                  entry["key"].startswith("gemma-3-27b-it/"))]
        if before != len(data["error_log"]):
            changes.append(f"Removed {before - len(data['error_log'])} from error_log")
    
    # Save
    with open(path, "w") as f:
        json.dump(data, f, indent=2)
    
    if changes:
        print(f"{model}: {' | '.join(changes)}")
    else:
        print(f"{model}: No changes needed")

print("\nDone! gemma-3-27b-it has been removed from all progress.json files")