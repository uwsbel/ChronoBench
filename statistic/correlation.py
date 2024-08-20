import matplotlib.pyplot as plt
import pandas as pd
from scipy.stats import spearmanr
import matplotlib.pyplot as plt
import seaborn as sns



# New metric data (Compile @1)
metric_compile = {
    "codestral-22b-instruct-v0.1": 21,
    "gemma-2-27b-it": 21,
    "gemma-2-9b-it": 21,
    "gpt-4o-mini": 20,
    "gemma-2-2b-it": 20,
    "llama-3.1-405b-instruct": 20,
    "mixtral-8x22b-instruct-v0.1": 20,
    "mistral-nemo-12b-instruct": 18,
    "mamba-codestral-7b-v0.1": 16,
    "mixtral-8x7b-instruct-v0.1": 15,
    "llama-3.1-8b-instruct": 14,
    "phi-3-mini-128k-instruct": 11,
    "llama-3.1-70b-instruct": 9,
    "mistral-large-latest": 9,
    "Gemini": 8,
    "nemotron-4-340b-instruct": 8,
    "claude-3-5-sonnet": 8,
    "phi-3-medium-128k-instruct": 6,
    "gpt-4o": 2
}

#pass@1 document
metric_pass = {
    "gpt-4o-mini": 13,
    "codestral-22b-instruct-v0.1": 10,
    "mixtral-8x22b-instruct-v0.1": 8,
    "claude-3-5-sonnet": 7,
    "Gemini": 7,
    "mixtral-8x7b-instruct-v0.1": 7,
    "llama-3.1-405b-instruct": 7,
    "mistral-nemo-12b-instruct": 7,
    "llama-3.1-8b-instruct": 5,
    "gemma-2-27b-it": 5,
    "llama-3.1-70b-instruct": 4,
    "gemma-2-9b-it": 4,
    "mamba-codestral-7b-v0.1": 3,
    "mistral-large-latest": 3,
    "gemma-2-2b-it": 3,
    "nemotron-4-340b-instruct": 3,
    "phi-3-medium-128k-instruct": 3,
    "phi-3-mini-128k-instruct": 3,
    "gpt-4o": 2
}

#ref
metric_ref_full = {
    "mixtral-8x7b-instruct-v0.1": 34.431373,
    "gpt-4o-mini": 34.137255,
    "mixtral-8x22b-instruct-v0.1": 33.784314,
    "llama-3.1-70b-instruct": 33.333333,
    "mistral-large-latest": 33.284314,
    "llama-3.1-405b-instruct": 33.088235,
    "Gemini": 32.941176,
    "codestral-22b-instruct-v0.1": 32.431373,
    "llama-3.1-8b-instruct": 31.117647,
    "nemotron-4-340b-instruct": 30.401961,
    "mistral-nemo-12b-instruct": 29.745098,
    "mamba-codestral-7b-v0.1": 29.421569,
    "gemma-2-9b-it": 29.392157,
    "gemma-2-27b-it": 29.264706,
    "gpt-4o": 28.284314,
    "gemma-2-2b-it": 27.450980,
    "claude-3-5-sonnet": 25.852941,
    "phi-3-mini-128k-instruct": 24.637255,
    "phi-3-medium-128k-instruct": 19.990196
}
#ref_doc
metric_ref_doc_full = {
    "gpt-4o-mini": 41.607843,
    "llama-3.1-70b-instruct": 39.686275,
    "Gemini": 39.205882,
    "llama-3.1-405b-instruct": 39.039216,
    "mixtral-8x22b-instruct-v0.1": 38.882353,
    "codestral-22b-instruct-v0.1": 37.862745,
    "mixtral-8x7b-instruct-v0.1": 37.715686,
    "llama-3.1-8b-instruct": 37.627451,
    "mistral-large-latest": 37.500000,
    "gemma-2-27b-it": 37.382353,
    "mistral-nemo-12b-instruct": 36.421569,
    "nemotron-4-340b-instruct": 35.088235,
    "gpt-4o": 33.901961,
    "claude-3-5-sonnet": 33.745098,
    "gemma-2-9b-it": 33.460784,
    "mamba-codestral-7b-v0.1": 31.833333,
    "gemma-2-2b-it": 31.068627,
    "phi-3-mini-128k-instruct": 27.764706,
    "phi-3-medium-128k-instruct": 22.009804
}
#doc
metric_doc_full = {
    "claude-3-5-sonnet": 52.784314,
    "gpt-4o": 49.362745,
    "mistral-large-latest": 43.931373,
    "gpt-4o-mini": 43.656863,
    "Gemini": 42.705882,
    "nemotron-4-340b-instruct": 39.872549,
    "codestral-22b-instruct-v0.1": 39.696078,
    "llama-3.1-70b-instruct": 39.441176,
    "llama-3.1-405b-instruct": 38.029412,
    "mixtral-8x22b-instruct-v0.1": 36.862745,
    "gemma-2-27b-it": 36.549020,
    "llama-3.1-8b-instruct": 34.509804,
    "mistral-nemo-12b-instruct": 31.813725,
    "mixtral-8x7b-instruct-v0.1": 30.735294,
    "gemma-2-9b-it": 30.441176,
    "mamba-codestral-7b-v0.1": 27.941176,
    "gemma-2-2b-it": 24.372549,
    "phi-3-mini-128k-instruct": 23.813725,
    "phi-3-medium-128k-instruct": 19.705882
}


#RougeLSum
metrics_RougeLSum_full = {
    "gpt-4o": 0.758101,
    "claude-3-5-sonnet": 0.756826,
    "mistral-large-latest": 0.739740,
    "Gemini": 0.726743,
    "gpt-4o-mini": 0.723202,
    "llama-3.1-405b-instruct": 0.719571,
    "nemotron-4-340b-instruct": 0.718667,
    "gemma-2-27b-it": 0.709674,
    "llama-3.1-70b-instruct": 0.709621,
    "codestral-22b-instruct-v0.1": 0.698970,
    "gemma-2-9b-it": 0.688353,
    "mixtral-8x22b-instruct-v0.1": 0.667283,
    "llama-3.1-8b-instruct": 0.655880,
    "mistral-nemo-12b-instruct": 0.648325,
    "gemma-2-2b-it": 0.640665,
    "mixtral-8x7b-instruct-v0.1": 0.624462,
    "phi-3-mini-128k-instruct": 0.582184,
    "mamba-codestral-7b-v0.1": 0.567418,
    "phi-3-medium-128k-instruct": 0.396457
}

#CodeBleu
metric_CodeBleu_full = {
    "llama-3.1-405b-instruct": 0.618578,
    "gpt-4o-mini": 0.609678,
    "Gemini": 0.608879,
    "codestral-22b-instruct-v0.1": 0.608012,
    "gpt-4o": 0.606506,
    "mistral-large-latest": 0.602497,
    "nemotron-4-340b-instruct": 0.597930,
    "claude-3-5-sonnet": 0.590604,
    "gemma-2-27b-it": 0.590290,
    "mixtral-8x22b-instruct-v0.1": 0.588542,
    "gemma-2-9b-it": 0.575422,
    "llama-3.1-8b-instruct": 0.571073,
    "mistral-nemo-12b-instruct": 0.551508,
    "gemma-2-2b-it": 0.542154,
    "llama-3.1-70b-instruct": 0.539998,
    "mixtral-8x7b-instruct-v0.1": 0.507564,
    "phi-3-mini-128k-instruct": 0.501759,
    "mamba-codestral-7b-v0.1": 0.473450,
    "phi-3-medium-128k-instruct": 0.375612
}

metric_ref_23 = {
    "mixtral-8x22b-instruct-v0.1": 44.970588,
    "mixtral-8x7b-instruct-v0.1": 44.764706,
    "gpt-4o-mini": 44.750000,
    "llama-3.1-405b-instruct": 44.588235,
    "Gemini": 44.544118,
    "llama-3.1-70b-instruct": 44.397059,
    "mistral-large-latest": 43.676471,
    "codestral-22b-instruct-v0.1": 43.029412,
    "llama-3.1-8b-instruct": 41.558824,
    "nemotron-4-340b-instruct": 41.102941,
    "mistral-nemo-12b-instruct": 38.779412,
    "mamba-codestral-7b-v0.1": 38.279412,
    "gemma-2-27b-it": 37.823529,
    "gemma-2-9b-it": 37.470588,
    "gemma-2-2b-it": 37.397059,
    "gpt-4o": 35.911765,
    "claude-3-5-sonnet": 32.926471,
    "phi-3-mini-128k-instruct": 31.264706,
    "phi-3-medium-128k-instruct": 25.544118
}

metric_doc_23= {
    "claude-3-5-sonnet": 50.191176,
    "gpt-4o": 47.147059,
    "gpt-4o-mini": 46.897059,
    "llama-3.1-8b-instruct": 44.852941,
    "Gemini": 44.617647,
    "mistral-large-latest": 43.911765,
    "llama-3.1-70b-instruct": 42.617647,
    "llama-3.1-405b-instruct": 42.250000,
    "codestral-22b-instruct-v0.1": 41.661765,
    "mixtral-8x22b-instruct-v0.1": 38.485294,
    "mixtral-8x7b-instruct-v0.1": 37.808824,
    "gemma-2-27b-it": 37.235294,
    "nemotron-4-340b-instruct": 37.235294,
    "mistral-nemo-12b-instruct": 36.970588,
    "gemma-2-9b-it": 32.779412,
    "mamba-codestral-7b-v0.1": 31.132353,
    "gemma-2-2b-it": 29.338235,
    "phi-3-mini-128k-instruct": 28.852941,
    "phi-3-medium-128k-instruct": 21.867647
}

metric_ref_doc_23 = {
    "gpt-4o-mini": 52.84,
    "llama-3.1-405b-instruct": 50.41,
    "llama-3.1-70b-instruct": 49.24,
    "Gemini": 49.00,
    "llama-3.1-8b-instruct": 48.69,
    "codestral-22b-instruct-v0.1": 48.60,
    "mixtral-8x22b-instruct-v0.1": 48.60,
    "mixtral-8x7b-instruct-v0.1": 48.38,
    "gemma-2-27b-it": 45.88,
    "mistral-large-latest": 45.87,
    "mistral-nemo-12b-instruct": 45.07,
    "nemotron-4-340b-instruct": 44.31,
    "gpt-4o": 40.88,
    "gemma-2-9b-it": 40.71,
    "mamba-codestral-7b-v0.1": 40.59,
    "gemma-2-2b-it": 39.85,
    "claude-3-5-sonnet": 39.54,
    "phi-3-mini-128k-instruct": 33.51,
    "phi-3-medium-128k-instruct": 26.65
}

metrics_RougeLSum23  = {
    "llama-3.1-405b-instruct": 0.922423,
    "gpt-4o-mini": 0.919854,
    "mistral-large-latest": 0.910594,
    "gpt-4o": 0.909050,
    "llama-3.1-70b-instruct": 0.906749,
    "codestral-22b-instruct-v0.1": 0.906151,
    "Gemini": 0.905613,
    "mixtral-8x22b-instruct-v0.1": 0.897707,
    "claude-3-5-sonnet": 0.894635,
    "nemotron-4-340b-instruct": 0.887252,
    "gemma-2-27b-it": 0.886780,
    "gemma-2-9b-it": 0.876468,
    "llama-3.1-8b-instruct": 0.873419,
    "mistral-nemo-12b-instruct": 0.840132,
    "gemma-2-2b-it": 0.838869,
    "mixtral-8x7b-instruct-v0.1": 0.818416,
    "phi-3-mini-128k-instruct": 0.773441,
    "mamba-codestral-7b-v0.1": 0.749815,
    "phi-3-medium-128k-instruct": 0.481481
}

metric_CodeBleu23 = {
    "llama-3.1-405b-instruct": 0.808603,
    "codestral-22b-instruct-v0.1": 0.797620,
    "gpt-4o-mini": 0.793188,
    "mixtral-8x22b-instruct-v0.1": 0.791093,
    "Gemini": 0.781381,
    "mistral-large-latest": 0.770280,
    "nemotron-4-340b-instruct": 0.768456,
    "gpt-4o": 0.766488,
    "gemma-2-27b-it": 0.762483,
    "llama-3.1-8b-instruct": 0.756116,
    "gemma-2-9b-it": 0.751777,
    "claude-3-5-sonnet": 0.734976,
    "mistral-nemo-12b-instruct": 0.714275,
    "gemma-2-2b-it": 0.710655,
    "llama-3.1-70b-instruct": 0.690767,
    "mixtral-8x7b-instruct-v0.1": 0.668471,
    "phi-3-mini-128k-instruct": 0.652963,
    "mamba-codestral-7b-v0.1": 0.625022,
    "phi-3-medium-128k-instruct": 0.470314
}


# Combine all metrics into a DataFrame
df = pd.DataFrame({
    "Pass@1": pd.Series(metric_pass),
    "Compile@1": pd.Series(metric_compile),
    "J-LLM_ref_doc": pd.Series(metric_ref_doc_full),
    "J-LLM_ref": pd.Series(metric_ref_full),
    "J-LLM_doc": pd.Series(metric_doc_full),
    "CodeBleu": pd.Series(metric_CodeBleu_full),
"RougeLSum": pd.Series(metrics_RougeLSum_full),
    #"LLM_ref_23": pd.Series(metric_ref_23),
    #"LLM_doc_23": pd.Series(metric_doc_23),
    #"LLM_ref_doc_23": pd.Series(metric_ref_doc_23),
    #"RougeLSum2": pd.Series(metrics_RougeLSum23),
    #"CodeBleu2": pd.Series(metric_CodeBleu23),

})

# Calculate the correlation matrix
correlation_matrix = df.corr(method='spearman')

# Display the correlation matrix
print(correlation_matrix)

# Set font size for all plot elements
plt.rcParams.update({'font.size': 20})  # Update font size
plt.rcParams['font.weight'] = 'bold'   # Set font weight to bold
plt.figure(figsize=(15, 15), dpi=300)
sns.heatmap(correlation_matrix, annot=True, cmap="coolwarm", vmin=-1, vmax=1, cbar=True, annot_kws={"size": 28})

# Set larger font sizes for title and labels
#plt.title("Spearman Correlation Matrix Between Metrics", fontsize=20)
#plt.xlabel('Metric', fontsize=18)
#plt.ylabel('Metric', fontsize=18)
plt.tight_layout()
plt.savefig('correlation_matrix.png')

plt.show()