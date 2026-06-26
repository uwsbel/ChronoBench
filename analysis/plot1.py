import pandas as pd
import matplotlib.pyplot as plt

# Load the data from the uploaded files
filtered_pass_data = pd.read_csv('filtered_pass_data.csv')
llm_data = pd.read_csv('LLM.csv')
pass_data = pd.read_csv('pass.csv')
# Set larger font sizes for all plot elements
plt.rcParams.update({'font.size': 16})  # General font size
plt.rcParams['axes.labelsize'] = 18     # Axis labels
plt.rcParams['axes.titlesize'] = 20     # Title font size
plt.rcParams['legend.fontsize'] = 14    # Legend font size
plt.rcParams['xtick.labelsize'] = 14    # X-axis tick labels
plt.rcParams['ytick.labelsize'] = 14    # Y-axis tick labels
# Aggregate the pass data across all rounds
pass_data['Total Passes'] = pass_data['1st Round Pass'] + pass_data['2nd & 3rd Rounds Pass']

# Summing up passes for each model
model_pass_summary = pass_data.groupby('Model Name')['Total Passes'].sum().reset_index()

# Rank the models based on the total passes
model_pass_summary['Rank'] = model_pass_summary['Total Passes'].rank(ascending=False, method='min')

# Calculate pass@1
total_tries = 34 * 3  # 34 systems and 3 rounds
model_pass_summary['pass@1'] = model_pass_summary['Total Passes'] / total_tries

# Calculate compile@1 for each model
pass_data['Total Compiles'] = pass_data['1st Round Compile'] + pass_data['2nd & 3rd Rounds Compile']

# Summing up compiles for each model
model_compile_summary = pass_data.groupby('Model Name')['Total Compiles'].sum().reset_index()

# Merge the compile data with the pass data
model_summary = model_pass_summary.merge(model_compile_summary, on='Model Name')

# Calculate compile@1
model_summary['compile@1'] = model_summary['Total Compiles'] / total_tries

# Sort by pass@1 for visualization
model_summary_sorted = model_summary.sort_values(by='pass@1', ascending=False).reset_index(drop=True)

# Remove the specific models from the dataset
models_to_remove = ['codellama-70b', 'phi-3-small-8k']
model_summary_filtered = model_summary_sorted[~model_summary_sorted['Model Name'].isin(models_to_remove)]

# Visualize the performance on pass@1 and compile@1 without the specified models
plt.figure(figsize=(6, 8), dpi=400)
# Add grid lines
plt.grid(True, which='both', linestyle='--', linewidth=0.5)
# Bar plot for pass@1
plt.barh(model_summary_filtered['Model Name'], model_summary_filtered['pass@1']*100, color='blue', label='pass@1')

# Bar plot for compile@1
plt.barh(model_summary_filtered['Model Name'], model_summary_filtered['compile@1']*100, color='orange', label='compile@1', alpha=0.6)
# Add labels and title
plt.xlabel('Performance (%)')
#
plt.ylabel('S-LLM')
#plt.title('Model Performance: pass@1 and compile@1')
plt.legend()
plt.gca().invert_yaxis()  # To display the highest ranked model at the top
plt.tight_layout()
plt.savefig('pass_compile.png')
plt.show()


# Aggregating the score data for the different metrics
score_columns = ['Score Reference Document', 'Score Reference', 'Score Document']
llm_scores = llm_data.groupby('Test Model')[score_columns].mean().reset_index()

# Sort by 'Score Reference Document' for visualization
llm_scores_sorted = llm_scores.sort_values(by='Score Reference Document', ascending=False).reset_index(drop=True)
# Clean up the model names by removing 'instruct', 'v0.1', and other similar suffixes
# Further clean up the model names for GPT models
llm_scores_sorted['Test Model'] = llm_scores_sorted['Test Model'].str.replace('gpt-4o-mini', 'GPT4o-mini', regex=False)
llm_scores_sorted['Test Model'] = llm_scores_sorted['Test Model'].str.replace('gpt-4o', 'GPT4o', regex=False)
llm_scores_sorted['Test Model'] = llm_scores_sorted['Test Model'].str.replace('-instruct', '', regex=False)
llm_scores_sorted['Test Model'] = llm_scores_sorted['Test Model'].str.replace('-v0.1', '', regex=False)
llm_scores_sorted['Test Model'] = llm_scores_sorted['Test Model'].str.replace('-latest', '', regex=False)
llm_scores_sorted['Test Model'] = llm_scores_sorted['Test Model'].str.replace('-it', '', regex=False)
# Visualize the performance on score_reference_document, score_reference, and score_document
plt.figure(figsize=(6,8), dpi=400)

# Bar plot for Score Reference Document
plt.barh(llm_scores_sorted['Test Model'], llm_scores_sorted['Score Reference Document'], color='green', label='J-LLM_Ref_Doc')

# Bar plot for Score Reference
plt.barh(llm_scores_sorted['Test Model'], llm_scores_sorted['Score Reference'], color='red', label='J-LLM_Ref', alpha=0.6)

# Bar plot for Score Document
plt.barh(llm_scores_sorted['Test Model'], llm_scores_sorted['Score Document'], color='purple', label='J-LLM_Doc', alpha=0.6)

# Add grid lines
plt.grid(True, which='both', linestyle='--', linewidth=0.5)

# Add labels and title
plt.xlabel('S-LLM Scores')
plt.ylabel('S-LLM')
#plt.title('LLM Performance: Score Reference Document, Score Reference, and Score Document')
plt.legend()
plt.gca().invert_yaxis()  # To display the highest ranked model at the top
# Use tight layout
plt.tight_layout()
plt.savefig('llm_scores.png')

# Display the plot
plt.show()



