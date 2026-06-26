# Re-import necessary libraries and re-load the score data and filter for second and third rounds only
import pandas as pd

# Reload the score data
score_reference_path = 'LLM.csv'
score_df = pd.read_csv(score_reference_path)

# Filter data for the second and third rounds only
filtered_rounds_df = score_df[score_df['Round'].isin(['second', 'third'])]

# Calculate the average of 'Score Reference' and 'Score Document' for each model, for the second and third rounds
#filtered_rounds_df['Combined Score'] = filtered_rounds_df[['Score Reference', 'Score Document']].mean(axis=1)

# Rank the models based on the combined score
ranked_ref_df = filtered_rounds_df.groupby('Test Model').agg({
    'Score Reference': 'mean'
}).reset_index()

ranked_doc_df=filtered_rounds_df.groupby('Test Model').agg({
    'Score Document': 'mean'
}).reset_index()

# Sort by Combined Score
ranked_ref_score_df = ranked_ref_df.sort_values(by='Score Reference', ascending=False).reset_index(drop=True)
ranked_doc_score_df=ranked_doc_df.sort_values(by='Score Document', ascending=False).reset_index(drop=True)
# Add ranking
ranked_ref_score_df['Rank'] = ranked_ref_score_df.index + 1
ranked_doc_score_df['Rank'] = ranked_doc_score_df.index + 1

print(ranked_ref_score_df[['Rank', 'Test Model', 'Score Reference']])
print(ranked_doc_score_df[['Rank', 'Test Model', 'Score Document']])