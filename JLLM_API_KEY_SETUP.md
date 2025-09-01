# JLLM API Key Configuration

## Overview
Each JLLM evaluation script can use a specific OpenAI API key to allow parallel evaluation and avoid rate limits.

## Environment Variables

Add these to your `.env` file:

### Model-Specific Keys (Recommended)
```bash
# Primary keys for each judge model
OPENAI_API_KEY_GPT4OMINI="your-api-key-for-gpt4o-mini"
OPENAI_API_KEY_GPT41MINI="your-api-key-for-gpt41-mini"  
OPENAI_API_KEY_GPT41NANO="your-api-key-for-gpt41-nano"
```

### Numbered Keys (Legacy Support)
```bash
# Alternative numbered keys (still supported)
OPENAI_API_KEY_1="your-api-key-1"  # Used by gpt-4o-mini if GPT4OMINI not set
OPENAI_API_KEY_2="your-api-key-2"  # Used by gpt-4.1-mini if GPT41MINI not set
OPENAI_API_KEY_3="your-api-key-3"  # Used by gpt-4.1-nano if GPT41NANO not set
```

### Default Fallback
```bash
# Default key used if specific keys are not found
OPENAI_API_KEY="your-default-api-key"
```

## Priority Order

Each script checks for API keys in this order:
1. **Model-specific key** (e.g., `OPENAI_API_KEY_GPT4OMINI`)
2. **Numbered key** (e.g., `OPENAI_API_KEY_1`)
3. **Default key** (`OPENAI_API_KEY`)

## Script Mapping

| Script | Primary Key | Secondary Key | Default |
|--------|------------|---------------|---------|
| `p_JLLM_score_gpt4omini.py` | `OPENAI_API_KEY_GPT4OMINI` | `OPENAI_API_KEY_1` | `OPENAI_API_KEY` |
| `p_JLLM_score_gpt41mini.py` | `OPENAI_API_KEY_GPT41MINI` | `OPENAI_API_KEY_2` | `OPENAI_API_KEY` |
| `p_JLLM_score_gpt41nano.py` | `OPENAI_API_KEY_GPT41NANO` | `OPENAI_API_KEY_3` | `OPENAI_API_KEY` |

## Benefits

1. **Parallel Execution**: Run multiple JLLM evaluations simultaneously
2. **Rate Limit Management**: Distribute API calls across different keys
3. **Cost Tracking**: Monitor usage per judge model
4. **Flexibility**: Easy to switch between keys without code changes

## Example Setup

```bash
# .env file
OPENAI_API_KEY="sk-default-key-for-general-use"
OPENAI_API_KEY_GPT4OMINI="sk-specific-key-for-gpt4o-mini"
OPENAI_API_KEY_GPT41MINI="sk-specific-key-for-gpt41-mini"
OPENAI_API_KEY_GPT41NANO="sk-specific-key-for-gpt41-nano"
```

## Verification

The scripts will log which key they're using:
- ✅ `Using OPENAI_API_KEY_GPT4OMINI for gpt-4o-mini evaluations`
- ⚠️ `OPENAI_API_KEY_GPT4OMINI and OPENAI_API_KEY_1 not found, using default OPENAI_API_KEY`

## Output Directories

Each judge now saves to its own directory to prevent overwriting:
- `gpt-4o-mini`: `/home/hongyu/Documents/SimBench/output_llms_gpt-4o-mini/`
- `gpt-4.1-mini`: `/home/hongyu/Documents/SimBench/output_llms_gpt-4-1-mini/`
- `gpt-4.1-nano`: `/home/hongyu/Documents/SimBench/output_llms_gpt-4-1-nano/`