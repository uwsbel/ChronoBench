Jingquan to drop in stuff here.

clean_truth.py is the function to remove the comments of the ground truth demos under the demo_data folder. Then the cleaned truth py files can be used to do similarity based evaluation.

extractPy.py is the function to remove the comments and extract the python code from S-LLM outputs.

evaluatePy.py is the function to compile and run the genereated python scripts from S-LLMs. 

p_sim_score.py is a parellel implementation to calculate the similarity-based baseline metrics, rougle family and codebleu.

