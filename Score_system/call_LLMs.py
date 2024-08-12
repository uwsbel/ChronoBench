from openai import OpenAI

# Define the API key outside the function
API_KEY = "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"
# Dictionary mapping model names to their API model identifiers
model_links = {
    "gemma-2-9b-it": "google/gemma-2-9b-it",
    "llama-405b": "meta/llama-3.1-405b-instruct",
}


def call_llm(model_name, user_input):
    global API_KEY  # Access the global API_KEY variable

    # Ensure the model name exists in the dictionary
    if model_name not in model_links:
        raise ValueError(f"Model name '{model_name}' not found in the model_links dictionary.")

    # Get the API model identifier
    model_identifier = model_links[model_name]

    client = OpenAI(
        base_url="https://integrate.api.nvidia.com/v1",
        api_key=API_KEY
    )

    completion = client.chat.completions.create(
        model=model_identifier,
        messages=[{"role": "user", "content": user_input}],
        temperature=0.2,
        top_p=0.7,
        max_tokens=1024,
        stream=True
    )

    for chunk in completion:
        if chunk.choices[0].delta.content is not None:
            print(chunk.choices[0].delta.content, end="")


# Example usage
call_llm("gemma-2-9b-it", "Write a limerick about the wonders of GPU computing.")
