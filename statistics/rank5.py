import pandas as pd


file_path = 'LLM.csv'
df = pd.read_csv(file_path)


# Rank the models based on 'Score Reference'
ranked_score_df = df.groupby('Test Model').agg({
    'Score Document': 'mean'
}).reset_index()

# Sort by Score Reference
ranked_score_df = ranked_score_df.sort_values(by='Score Document', ascending=False).reset_index(drop=True)

# Add ranking
ranked_score_df['Rank'] = ranked_score_df.index + 1

print(ranked_score_df[['Rank', 'Test Model', 'Score Document']])
