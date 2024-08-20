import pandas as pd
import matplotlib.pyplot as plt
# Load the CSV files
file_path = 'D:\SimBench\statistic\\filtered_pass_data.csv'
data = pd.read_csv(file_path)

llm_file_path = 'D:\SimBench\statistic\LLM.csv'
llm_data = pd.read_csv(llm_file_path)



import matplotlib.pyplot as plt

# Group the data by 'Model Name' and sum the '1st Round Pass' and '2nd & 3rd Rounds Pass' values
performance_data = data.groupby('Model Name').sum()[['1st Round Pass', '2nd & 3rd Rounds Pass']]

# Plotting the performance
plt.figure(figsize=(10, 6), dpi=1000)
performance_data.plot(kind='bar', stacked=True, color=['skyblue', 'orange'])

plt.title('Performance of LLMs for Passing Criteria')
plt.xlabel('Model Name')
plt.ylabel('Number of Passes')
plt.legend(['1st Round Pass', '2nd & 3rd Rounds Pass'])
plt.xticks(rotation=45, ha='right')
plt.tight_layout()

print(performance_data)
# Group the data by 'Model Name' and sum the '1st Round Compile' and '2nd & 3rd Rounds Compile' values
filtered_compile_data = data.groupby('Model Name').sum()[['1st Round Compile', '2nd & 3rd Rounds Compile']]

# Plotting the compile numbers
plt.figure(figsize=(10, 6))
filtered_compile_data.plot(kind='bar', stacked=True, color=['lightgreen', 'salmon'])

plt.title('Compilation Success of LLMs')
plt.xlabel('Model Name')
plt.ylabel('Number of Successful Compilations')
plt.legend(['1st Round Compile', '2nd & 3rd Rounds Compile'])
plt.xticks(rotation=45, ha='right')
plt.tight_layout()


# Pivot the data to show scores for each model across the different rounds
pivot_table = llm_data.pivot_table(index='Test Model', columns='Round',
                                   values=['Score Document', 'Score Reference', 'Score Reference Document'])

# Plot the scores for each model across the different rounds
plt.figure(figsize=(12, 8))

# Plot the 'Score Reference' and 'Score Reference Document' for each model across the different rounds
plt.figure(figsize=(12, 8))

pivot_table[['Score Reference', 'Score Reference Document']].plot(kind='bar', stacked=False, ax=plt.gca(), width=0.8)
plt.title('Score Reference and Score Reference Document for Each Model Across Rounds')
plt.xlabel('Test Model')
plt.ylabel('Scores')
plt.xticks(rotation=45, ha='right')
plt.legend(title='Round and Score Type', bbox_to_anchor=(1.05, 1), loc='upper left')
plt.tight_layout()

# Pivot the data to show scores for each model across the different rounds
pivot_table = llm_data.pivot_table(index='Test Model', columns='Round',
                                   values=['Score Document', 'Score Reference', 'Score Reference Document'])

# Plot the scores for each model across the different rounds
plt.figure(figsize=(12, 8))

pivot_table['Score Document'].plot(kind='bar', stacked=False, ax=plt.gca(), width=0.8)
plt.title('Scores for Each Model Across Rounds')
plt.xlabel('Test Model')
plt.ylabel('Scores')
plt.xticks(rotation=45, ha='right')
plt.legend(title='Round and Score Type', bbox_to_anchor=(1.05, 1), loc='upper left')
plt.tight_layout()


# Plot the 'Score Reference' and 'Score Reference Document' for each model across the different rounds
plt.figure(figsize=(12, 8))

pivot_table[['Score Reference', 'Score Reference Document']].plot(kind='bar', stacked=False, ax=plt.gca(), width=0.8)
plt.title('Score Reference and Score Reference Document for Each Model Across Rounds')
plt.xlabel('Test Model')
plt.ylabel('Scores')
plt.xticks(rotation=45, ha='right')
plt.legend(title='Round and Score Type', bbox_to_anchor=(1.05, 1), loc='upper left')
plt.tight_layout()



plt.show()