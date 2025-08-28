#!/bin/bash

# Helper script to set up multiple OpenAI API keys
# This enables parallel processing of OpenAI judge models

echo "=========================================="
echo "Multi-API Key Setup for Parallel Processing"
echo "=========================================="
echo ""
echo "This script helps you configure multiple OpenAI API keys"
echo "to enable parallel evaluation with different judge models."
echo ""

ENV_FILE="../.env"

# Check if .env exists
if [ ! -f "$ENV_FILE" ]; then
    echo "Creating new .env file..."
    touch "$ENV_FILE"
fi

# Function to add or update an environment variable
update_env() {
    local key=$1
    local value=$2
    
    if grep -q "^$key=" "$ENV_FILE"; then
        # Update existing key
        sed -i "s|^$key=.*|$key=\"$value\"|" "$ENV_FILE"
        echo "✓ Updated $key"
    else
        # Add new key
        echo "$key=\"$value\"" >> "$ENV_FILE"
        echo "✓ Added $key"
    fi
}

echo "Current OpenAI API key configuration:"
echo "--------------------------------------"

# Check existing keys
if grep -q "^OPENAI_API_KEY=" "$ENV_FILE"; then
    current_key=$(grep "^OPENAI_API_KEY=" "$ENV_FILE" | cut -d'"' -f2)
    echo "Primary key: ${current_key:0:10}..."
else
    echo "Primary key: Not set"
fi

for i in 1 2 3; do
    if grep -q "^OPENAI_API_KEY_$i=" "$ENV_FILE"; then
        current_key=$(grep "^OPENAI_API_KEY_$i=" "$ENV_FILE" | cut -d'"' -f2)
        echo "API Key #$i: ${current_key:0:10}..."
    else
        echo "API Key #$i: Not set"
    fi
done

echo ""
echo "--------------------------------------"
echo "Enter your OpenAI API keys (or press Enter to skip):"
echo ""

# Collect API keys
for i in 1 2 3; do
    echo -n "Enter OpenAI API Key #$i (sk-...): "
    read -r api_key
    
    if [ -n "$api_key" ]; then
        update_env "OPENAI_API_KEY_$i" "$api_key"
        
        # Also set as primary if it's the first one
        if [ $i -eq 1 ] && ! grep -q "^OPENAI_API_KEY=" "$ENV_FILE"; then
            update_env "OPENAI_API_KEY" "$api_key"
        fi
    else
        echo "⏭ Skipped API Key #$i"
    fi
done

echo ""
echo "=========================================="
echo "Configuration Complete!"
echo "=========================================="

# Check how many unique keys we have
if [ -f "$ENV_FILE" ]; then
    key1=$(grep "^OPENAI_API_KEY_1=" "$ENV_FILE" 2>/dev/null | cut -d'"' -f2)
    key2=$(grep "^OPENAI_API_KEY_2=" "$ENV_FILE" 2>/dev/null | cut -d'"' -f2)
    key3=$(grep "^OPENAI_API_KEY_3=" "$ENV_FILE" 2>/dev/null | cut -d'"' -f2)
    
    unique_keys=0
    [ -n "$key1" ] && unique_keys=$((unique_keys + 1))
    [ -n "$key2" ] && [ "$key2" != "$key1" ] && unique_keys=$((unique_keys + 1))
    [ -n "$key3" ] && [ "$key3" != "$key1" ] && [ "$key3" != "$key2" ] && unique_keys=$((unique_keys + 1))
    
    if [ $unique_keys -gt 1 ]; then
        echo "✓ $unique_keys unique API keys configured"
        echo "✓ Parallel processing ENABLED for OpenAI models"
        echo ""
        echo "Benefits:"
        echo "  • ${unique_keys}x faster evaluation"
        echo "  • Each key has separate rate limits"
        echo "  • Automatic load balancing"
    elif [ $unique_keys -eq 1 ]; then
        echo "✓ 1 API key configured"
        echo "⚠ Sequential processing will be used (single key)"
        echo ""
        echo "To enable parallel processing, add more unique API keys."
    else
        echo "⚠ No API keys configured"
        echo "Please add at least one OpenAI API key to .env"
    fi
fi

echo ""
echo "To run evaluations:"
echo "  cd /home/hongyu/Documents/SimBench/scoring"
echo "  ./run_multiple_judges.sh"
echo ""