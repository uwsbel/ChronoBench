# Re-execute all steps to properly filter, rank, and sort based on CodeBleu
import pandas as pd

# Reload the initial file with CodeBleu data
file_path = 'evaluation_results.csv'
df = pd.read_csv(file_path)

# Filter data for the second and third rounds only
filtered_codebleu_df = df[df['round'].isin(['round_2', 'round_3'])]

# Rank the models based on CodeBleu scores for the second and third rounds
ranked_codebleu_df = filtered_codebleu_df.groupby('model').agg({
    'codebleu': 'mean'
}).reset_index()

# Sort by CodeBleu
ranked_codebleu_df = ranked_codebleu_df.sort_values(by='codebleu', ascending=False).reset_index(drop=True)

# Add ranking
ranked_codebleu_df['Rank'] = ranked_codebleu_df.index + 1

print(ranked_codebleu_df[['Rank', 'model', 'codebleu']])
