import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

# Load the two CSV files
file_1_path = 'D:\SimBench\statistic\\filtered_pass_data.csv'
file_2_path = 'D:\SimBench\statistic\\LLM.csv'

df1 = pd.read_csv(file_1_path)
df2 = pd.read_csv(file_2_path)

# Display the first few rows of each dataframe (optional)
print(df1.head())
print(df2.head())

# Group the data by 'Model Name' and sum up the pass counts for each model
df1_grouped = df1.groupby('Model Name').sum()

# Improved version of the previous chart
plt.figure(figsize=(16, 10))  # Increase the figure size

# Use a color palette
colors = sns.color_palette('viridis', len(df1_grouped.index))

# Create the bar plot with the updated appearance
sns.barplot(
    x=df1_grouped.index,
    y=df1_grouped['1st Round Pass'] + df1_grouped['2nd & 3rd Rounds Pass'],
    palette=colors
)

# Rotate x-axis labels for better readability
plt.xticks(rotation=45, ha="right")

# Add a grid for better readability
plt.grid(axis='y', linestyle='--', alpha=0.7)

# Add data labels to each bar
for i in range(len(df1_grouped.index)):
    plt.text(
        i,
        df1_grouped['1st Round Pass'].values[i] + df1_grouped['2nd & 3rd Rounds Pass'].values[i] + 0.1,
        df1_grouped['1st Round Pass'].values[i] + df1_grouped['2nd & 3rd Rounds Pass'].values[i],
        ha='center',
        va='bottom',
        fontsize=10,
        color='black'
    )

#plt.title('Pass @ k', fontsize=16)
plt.xlabel('Model Name', fontsize=14)
plt.ylabel('Pass @ k', fontsize=20)

plt.show()