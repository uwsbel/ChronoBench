
import pandas as pd


file_path = 'evaluation_results.csv'
df = pd.read_csv(file_path)

# Filter data for the second and third rounds only
filtered_rougeLsum_df = df[df['round'].isin(['round_2', 'round_3'])]


ranked_rougeLsum_df = filtered_rougeLsum_df.groupby('model').agg({
    'rougeLsum': 'mean'
}).reset_index()


ranked_rougeLsum_df = ranked_rougeLsum_df.sort_values(by='rougeLsum', ascending=False).reset_index(drop=True)


ranked_rougeLsum_df['Rank'] = ranked_rougeLsum_df.index + 1

print(ranked_rougeLsum_df[['Rank', 'model', 'rougeLsum']])
