# JLLM Evaluation System Fixes Summary

## Issues Fixed

### 1. Identical Scores Across Different JLLMs ✅
**Problem:** All three JLLM scripts (gpt-4.1-nano, gpt-4.1-mini, gpt-4o-mini) were saving evaluations to the same `/output_llms/` directory, causing later scripts to overwrite earlier results.

**Solution:** Modified each script to use judge-specific output directories:
- `gpt-4o-mini` → `/output_llms_gpt-4o-mini/`
- `gpt-4.1-mini` → `/output_llms_gpt-4-1-mini/`
- `gpt-4.1-nano` → `/output_llms_gpt-4-1-nano/`

### 2. API Key Configuration ✅
**Problem:** All scripts were using the same API key, limiting parallel execution.

**Solution:** Updated each script to use model-specific environment variables:
- `p_JLLM_score_gpt4omini.py`: Uses `OPENAI_API_KEY_GPT4OMINI` (fallback: `OPENAI_API_KEY_1`)
- `p_JLLM_score_gpt41mini.py`: Uses `OPENAI_API_KEY_GPT41MINI` (fallback: `OPENAI_API_KEY_2`)
- `p_JLLM_score_gpt41nano.py`: Uses `OPENAI_API_KEY_GPT41NANO` (fallback: `OPENAI_API_KEY_3`)

### 3. gpt-4o-mini Skipping Evaluations ✅
**Problem:** The script was skipping all evaluations due to a `progress.json` file that incorrectly marked all 612 tasks as completed.

**Root Cause:** The progress tracker was persisting completion status even when actual score files didn't exist, likely from a previous interrupted run.

**Solution:** Renamed `progress.json` to `progress_backup.json` to force re-evaluation.

### 4. Score Extraction Warnings in gpt-4.1-nano ✅
**Problem:** Score extraction was failing with warnings like:
```
WARNING - No valid score found in /home/hongyu/Documents/SimBench/output_llms_gpt-4-1-nano/deepseek-r1/hmmwv/second_score_reference_document.txt
```

**Root Cause:** The JLLM was outputting scores in format `[[x]] 70` instead of the expected `[[70]]`.

**Solution:** Updated the `extract_scores_from_txt()` function in all three scripts to handle both formats:
```python
# Original format: [[70]]
match = re.search(r"\[\[(\d+)\]\]", content)
if match:
    return int(match.group(1))

# New format: [[x]] 70
match_x = re.search(r"\[\[x\]\]\s*(\d+)", content)
if match_x:
    score = int(match_x.group(1))
    return score
```

## Correct Evaluation Pipeline

### Phase 1: JLLM Evaluation
Each JLLM script independently evaluates model responses:

1. **Input**: Model responses from `/output_llms_{judge}/` directories
2. **Process**: Each JLLM evaluates responses against reference code
3. **Output**: 
   - Individual score files: `{round}_score_{type}.txt`
   - CSV per model/system: `evaluation_scores.csv`
   - Combined CSV: `combined_evaluation_scores_{judge}.csv`

### Phase 2: Score Merging
The `create_final_combined_scores.py` script merges JLLM scores with similarity metrics:

1. **Input**:
   - JLLM scores: `combined_evaluation_scores_{judge}.csv`
   - Similarity metrics: `evaluation_results.csv`
2. **Process**: Merges on (model, system) keys
3. **Output**: `final_combined_scores_{judge}.csv`

### Phase 3: Ranking Generation
The ranking script creates final rankings:

1. **Input**: `final_combined_scores_{judge}.csv`
2. **Process**: Computes rankings based on all metrics
3. **Output**: `all_scores_ranked.csv` in `/out_diff_models/out_{judge}/`

## Directory Structure
```
/home/hongyu/Documents/SimBench/
├── output_llms_gpt-4o-mini/        # gpt-4o-mini evaluations
├── output_llms_gpt-4-1-mini/       # gpt-4.1-mini evaluations  
├── output_llms_gpt-4-1-nano/       # gpt-4.1-nano evaluations
├── scoring/
│   ├── v01/
│   │   ├── p_JLLM_score_gpt4omini.py
│   │   ├── p_JLLM_score_gpt41mini.py
│   │   └── p_JLLM_score_gpt41nano.py
│   └── out_diff_models/
│       ├── out_gpt-4o-mini/
│       │   ├── combined_evaluation_scores_gpt-4o-mini.csv
│       │   ├── final_combined_scores_gpt-4o-mini.csv
│       │   └── all_scores_ranked.csv
│       ├── out_gpt-4-1-mini/
│       │   └── [similar structure]
│       └── out_gpt-4-1-nano/
│           └── [similar structure]
```

## Why gpt-4.1-nano Had Score Warnings
The gpt-4.1-nano model was outputting evaluation scores in a different format than expected. Instead of the standard `[[70]]` format, it was using `[[x]] 70` where `[[x]]` marks the score location and the actual number follows. This appears to be a model-specific quirk in how gpt-4.1-nano formats its evaluations. The fix now handles both formats gracefully.

## Verification Steps
1. Check that each JLLM has its own output directory
2. Verify API keys are correctly configured in `.env`
3. Ensure no `progress.json` files are blocking evaluations
4. Confirm score extraction handles both `[[70]]` and `[[x]] 70` formats
5. Validate that final rankings show unique scores per JLLM