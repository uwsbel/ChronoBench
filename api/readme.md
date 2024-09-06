This folder contains a well-designed API document `api.txt` used for the metrics `J-LLM_ref_doc` and `J-LLM_doc` mentioned in the paper. The API document was created through a two-level summarization process using an LLM:

1. **First-Level Summaries**: The LLM was used to summarize the API usage for individual demos in each class (e.g., "FEA", "MBS", "VEH"). This provided a clear understanding of how the APIs are applied within each domain.

2. **Final Summary**: All first-level summaries were then consolidated into a comprehensive document, `api.txt`. The final document is approximately 4,000 tokens in length when processed using GPT-4.

This structured approach ensures that the API document is concise, while capturing essential details for each demo and class, making it an effective reference for understanding API usage across different LLMs and simulations.
