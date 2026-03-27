# SimBench Multivariate Analysis Report

**Dataset:** `all_metrics_merged_pretrain_only.csv`  
**Script:** `scoring/multivariate_analysis.py`  
**Output directory:** `scoring/out/multivariate/`

---

## Dataset Overview

| Item | Value |
|------|-------|
| Total observations | 3,570 (35 models × 34 systems × 3 rounds) |
| Models evaluated | 35 (21 open-source, 12 closed-source, 2 unmatched) |
| Simulation systems | 34 across 5 categories |
| Interaction rounds | 3 (T1: initial, T2: error feedback, T3: extension) |
| Primary outcome | `score_reference_document` (J-LLM-Ref-Doc, 0–100) |

**Category composition:** SEN (4 systems, n=420), RBT (6, n=630), VEH (14, n=1470), MBS (5, n=525), FEA (5, n=525)

---

## Part I — Core Multivariate Analysis (Sections 1–9)

### 1. Descriptive Statistics

**Overall distribution:** mean = 37.7 ± 22.4, median = 32, IQR = [22, 55], range = [0, 100]

**By category (sorted by difficulty, hardest first):**

| Category | Mean | SD | Min | Max | N |
|----------|------|----|-----|-----|---|
| SEN | 31.28 | 20.10 | 0 | 92 | 420 |
| RBT | 35.77 | 20.83 | 0 | 91 | 630 |
| VEH | 38.02 | 22.94 | 0 | 100 | 1470 |
| MBS | 40.77 | 21.38 | 0 | 88 | 525 |
| FEA | 40.98 | 24.03 | 0 | 95 | 525 |

**By interaction round:**

| Round | Mean | SD | Description |
|-------|------|----|-------------|
| T1 | 20.22 | 11.57 | Initial generation |
| T2 | 49.47 | 21.32 | After error feedback (+29.3) |
| T3 | 43.31 | 20.96 | After extension request (−6.2) |

**Top-5 / Bottom-5 models:**

| Rank | Model | Mean Score |
|------|-------|-----------|
| 1 | claude-4-sonnet-20250514 | 49.18 |
| 2 | o3 | 46.32 |
| 3 | claude-3-7-sonnet-20250219 | 43.34 |
| 4 | qwen3-235b-a22b | 42.12 |
| 5 | o4-mini | 42.05 |
| 31 | mamba-codestral-7b-v0.1 | 31.83 |
| 32 | gemma-2-2b-it | 31.07 |
| 33 | phi-3-mini-128k-instruct | 27.76 |
| 34 | gemma-3-1b-it | 27.55 |
| 35 | phi-3-medium-128k-instruct | 22.01 |

---

### 2. Three-Factor ANOVA (Type III SS)

**Model:** `score ~ C(category) + C(round) + C(category):C(round)`

| Source | SS | df | F | p | η² |
|--------|----|----|---|---|----|
| Category | 1,165.5 | 4 | 0.90 | 0.464 (n.s.) | 0.0008 |
| **Round** | **130,074.7** | **2** | **200.6** | **<10⁻⁸³** | **0.093** |
| Category × Round | 38,688.9 | 8 | 14.9 | <10⁻²¹ | 0.028 |
| Residual | 1,152,381 | 3555 | — | — | — |

> **Key finding:** Round is the dominant source of variance (η²=0.093), an order of magnitude larger than the category main effect (η²=0.0008). Once round is controlled, categories are not significantly different (*p*=0.464). However, the significant Category × Round interaction (η²=0.028) reveals that different categories respond differently to multi-turn feedback.

**Simple one-way ANOVAs:**
- Category: F(4, 3565) = 15.43, p = 1.6×10⁻¹²
- Round: F(2, 3567) = 826.59, p ≈ 0

---

### 3. Post-hoc Comparisons (Tukey HSD, α = 0.05)

**Category pairwise comparisons:**

| Pair | Δ Mean | p-adj | Significant |
|------|--------|-------|-------------|
| FEA vs MBS | −0.21 | 0.9999 | ✗ |
| FEA vs VEH | −2.96 | 0.066 | ✗ |
| MBS vs VEH | −2.75 | 0.105 | ✗ |
| RBT vs VEH | +2.25 | 0.208 | ✗ |
| **FEA vs RBT** | **−5.21** | **0.0007** | **✓** |
| **FEA vs SEN** | **−9.70** | **<0.001** | **✓** |
| **MBS vs RBT** | **−5.00** | **0.001** | **✓** |
| **MBS vs SEN** | **−9.49** | **<0.001** | **✓** |
| **RBT vs SEN** | **−4.49** | **0.012** | **✓** |
| **SEN vs VEH** | **+6.74** | **<0.001** | **✓** |

> SEN is significantly harder than all other categories. FEA/MBS form one cluster; VEH/RBT form another; SEN stands alone as the most difficult.

**Round pairwise comparisons:** All three pairs are significantly different (*p* ≈ 0):
- T1 → T2: +29.26 points
- T2 → T3: −6.17 points
- T1 → T3: +23.09 points (net gain)

---

### 4. Metric Correlation Analysis

**Pearson r and Spearman ρ with J-LLM-Ref-Doc (score_reference_document):**

| Metric | Pearson r | Spearman ρ | Significance |
|--------|-----------|-----------|--------------|
| score_reference | +0.769 | +0.707 | *** |
| codebleu | +0.711 | +0.720 | *** |
| rouge2 | +0.688 | +0.736 | *** |
| rougeL | +0.690 | +0.727 | *** |
| rouge1 / rougeLsum | +0.674 | +0.740 | *** |
| ngram_match_score | +0.694 | +0.710 | *** |
| weighted_ngram | +0.692 | +0.713 | *** |
| dataflow_match | +0.602 | +0.593 | *** |
| syntax_match | +0.568 | +0.682 | *** |
| **score_document** | **+0.229** | **+0.240** | *** |

> `score_document` (J-LLM without reference answer) is a weak predictor of `score_reference_document`, confirming that reference-augmented evaluation captures qualitatively different information. CodeBLEU and ROUGE metrics are moderately correlated (~0.67–0.77), indicating they serve as useful but imperfect proxies.

---

### 5. PCA on Metric Space

**Explained variance:**

| PC | Variance | Cumulative | Dominant Loading |
|----|----------|-----------|-----------------|
| PC1 | 76.49% | 76.49% | All code metrics uniformly (~0.32) |
| PC2 | 8.46% | 84.95% | score_document (0.925) |
| PC3 | 6.33% | 91.28% | score_reference (0.659) |
| PC4 | 3.72% | 95.00% | dataflow_match (0.880) |

> Only **4 components** are needed to capture 95% of variance. PC1 represents a unified "code quality" dimension that conflates all text-similarity metrics. PC2 isolates the reference-free J-LLM signal, and PC3 isolates the reference-based J-LLM signal — these two are nearly orthogonal, explaining why score_document is a weak predictor of score_reference_document.

---

### 6. Linear Regression: Predicting J-LLM from CodeBLEU/ROUGE

**OLS: score_reference_document ~ all 9 code metrics**

| Metric | R² = 0.516 | adj-R² = 0.515 | F = 421.2 (p < 10⁻³⁰⁰) |
|--------|-----------|---------------|------------------------|

**Significant predictors:**

| Predictor | β | SE | t | p |
|-----------|---|----|---|---|
| codebleu | +51.63 | 8.38 | 6.16 | <0.001 |
| syntax_match_score | −19.82 | 3.14 | −6.31 | <0.001 |
| dataflow_match_score | +9.16 | 2.19 | 4.17 | <0.001 |

**R² by round — code metrics explain much less in T1:**

| Round | R² | adj-R² |
|-------|----|--------|
| T1 | 0.123 | 0.117 |
| T2 | 0.338 | 0.333 |
| T3 | 0.385 | 0.380 |

> At T1 (initial generation with low scores), code-similarity metrics are poor proxies for J-LLM judgment (R²=0.12). As code quality improves in T2/T3, the two metric families converge (R²=0.34–0.38). This suggests that CodeBLEU/ROUGE can substitute for J-LLM evaluation only at higher quality levels.

---

### 7. Category × Round Interaction Effects

**Mean J-LLM-Ref-Doc scores:**

| Category | T1 | T2 | T3 | Δ12 | Δ23 | Overall |
|----------|----|----|-----|------|------|---------|
| SEN | 19.1 | 39.5 | 35.3 | +20.4 | −4.2 | 31.3 |
| RBT | 20.3 | 51.4 | 35.5 | +31.1 | **−15.9** | 35.8 |
| VEH | 19.7 | 46.8 | **47.6** | +27.2 | **+0.7** | 38.0 |
| MBS | 22.4 | 53.4 | 46.5 | +31.0 | −6.9 | 40.8 |
| FEA | 20.4 | **58.6** | 44.0 | **+38.2** | −14.7 | 41.0 |

> - **VEH is the only category where T3 does not decline** (Δ23 = +0.7), suggesting vehicle simulations benefit from diverse extension patterns.
> - **FEA achieves the highest T2 score** (58.6) but suffers the steepest T3 drop (−14.7), indicating brittleness in extending finite element simulations beyond the corrected baseline.
> - **SEN has the weakest T1→T2 improvement** (+20.4), suggesting error feedback is least effective for sensor configuration tasks.
> - **RBT has the steepest T3 decline** (−15.9), indicating robotic simulation extensions are particularly fragile.

---

## Part II — Extended Model Metadata Analysis (Sections 10–13)

### 10. Open-Source vs. Closed-Source Comparison

**Model classification:** 21 open-source, 12 closed-source (Anthropic, OpenAI, Google Gemini)

| | Open-source | Closed-source |
|---|---|---|
| n | 21 | 12 |
| Mean score | 35.77 | **40.96** |
| SD | 5.23 | 4.40 |
| Range | [22.0, 42.1] | [33.7, 49.2] |

**Statistical tests:**
- Mann-Whitney U = 48, **p = 0.0037**
- Independent t(31) = −2.90, **p = 0.0068**
- **Cohen's d = 1.075 (large effect)**
- One-way ANOVA: F = 42.32, p < 10⁻¹⁰

> Closed-source models significantly outperform open-source models with a large effect size. The performance gap is consistent across all five simulation categories.

**Per-category gap (Closed − Open):**

| Category | Closed | Open | Gap |
|----------|--------|------|-----|
| SEN | 35.0 | 29.2 | **+5.8** |
| RBT | 37.6 | 34.4 | +3.2 |
| VEH | 40.8 | 36.5 | +4.3 |
| MBS | 45.4 | 38.2 | **+7.2** |
| FEA | 45.7 | 38.2 | **+7.6** |

> The advantage of closed-source models is largest for technically demanding categories (FEA +7.6, MBS +7.2) and smallest for robotics (RBT +3.2). This suggests that proprietary training data and RLHF tuning provide the greatest benefit for physics-intensive simulation tasks.

---

### 11. Model Size Effect (Open-Source Models)

**Sample:** 21 open-source models with known parameter counts, ranging from 1B to 405B.

**Correlation analysis:**

| Predictor | Pearson r | p | Spearman ρ | p |
|-----------|-----------|---|-----------|---|
| Raw size (B) | +0.457 | 0.037 | +0.792 | <0.001 |
| **log₁₀(size)** | **+0.702** | **0.0004** | **+0.792** | **<0.001** |

> The log-linear relationship is substantially stronger than the linear one (r=0.702 vs 0.457), confirming a **logarithmic scaling law**: each order-of-magnitude increase in parameters yields approximately **+4.8 score points**.

**OLS: score ~ log₁₀(size)**  R² = 0.493, β = 4.814 (p = 0.0004)

**By size bin:**

| Size Bin | n | Mean | SD |
|---------|---|------|----|
| Small (≤10B) | 7 | 31.9 | 3.6 |
| Mid (10–30B) | 4 | 33.4 | 7.6 |
| Large (30–100B) | 4 | 39.3 | 1.5 |
| XLarge (>100B) | 6 | 39.5 | 2.5 |

> Performance plateaus beyond ~30B parameters (39.3 for Large vs 39.5 for XLarge), suggesting **diminishing returns at scale** for this domain-specific coding task. The jump from Small to Large is substantial (+7.4 points), but further scaling beyond 30B yields minimal gains.

---

### 12. Release Date Effect

**All 33 models (Dec 2023 baseline):**
- Pearson r = +0.503 (p = 0.003)
- Spearman ρ = +0.645 (p = 0.0001)
- OLS: β = 0.0176 pts/day ≈ **+0.53 pts/month**

**Stratified by source type:**

| Group | n | Pearson r | p | Slope (pts/month) |
|-------|---|-----------|---|-------------------|
| All | 33 | +0.503 | 0.003 | +0.53 |
| **Closed-source** | **12** | **+0.656** | **0.021** | **+0.52** |
| Open-source | 21 | +0.289 | 0.205 (n.s.) | +0.33 |

> The temporal trend is **statistically significant only for closed-source models** (r=0.656, p=0.021). Open-source improvement over time is not significant at α=0.05 (r=0.289, p=0.20), possibly because open-source model releases are more heterogeneous in training objectives, with smaller models released throughout the period dampening the trend.

**By release year:**

| Year | n | Mean | SD |
|------|---|------|----|
| 2023 | 1 | 37.7 | — |
| 2024 | 18 | 35.4 | 5.0 |
| **2025** | **14** | **40.5** | **5.1** |

> 2025 models average **+5.1 points** above 2024 models.

---

### 13. Multi-Factor Regression on Model-Level Scores

**Dependent variable:** per-model mean J-LLM-Ref-Doc score  
**Predictors:** source type (binary), log₁₀(size) (imputed median for closed-source), days since Dec 2023

| Model | Predictors | n | R² | adj-R² | F | p |
|-------|-----------|---|----|----|---|---|
| M1 | source_type + date | 33 | 0.348 | 0.304 | 8.00 | 0.002 |
| **M2** | **source_type + date + log_size** | **33** | **0.595** | **0.553** | **14.17** | **<10⁻⁵** |
| M3 | log_size + date (open only) | 21 | 0.534 | 0.483 | 10.32 | 0.001 |

**M2 coefficients (best model):**

| Predictor | β | Interpretation |
|-----------|---|----------------|
| Intercept | 25.79 | baseline score |
| is_closed | +4.04 | closed-source premium |
| days_since_ref | +0.012 | +0.35 pts/month |
| log_size_imp | +4.54 | +4.5 pts per 10× size |

> **Three factors together explain 59.5% of inter-model variance (adj-R²=0.553).** Model size is the strongest single predictor among the three (based on standardized coefficients). When size is included, the source-type effect partially persists (+4.0 pts), suggesting that beyond scale, closed-source models benefit from additional proprietary advantages (training data quality, RLHF, system prompting).

---

## Summary of Key Findings

| # | Finding | Key Statistic |
|---|---------|--------------|
| 1 | **Round effect dominates all other factors** | η²=0.093, F=826.6, p<10⁻²⁹⁵ |
| 2 | **T2 (error feedback) is the most valuable interaction** | +29.3 pts over T1 (Tukey p≈0) |
| 3 | **SEN is the hardest category; FEA/MBS easiest** | SEN mean=31.3 vs FEA=41.0, p<0.001 |
| 4 | **VEH is uniquely robust at T3** | Δ23=+0.7 (only positive) |
| 5 | **Metric space is largely 1-dimensional (PC1=76%)** | 4 PCs cover 95% variance |
| 6 | **CodeBLEU is a moderate but imperfect J-LLM proxy** | r=0.711 overall; R²=0.12 at T1 |
| 7 | **Closed-source significantly outperforms open-source** | d=1.075, p=0.004 |
| 8 | **Log-linear scaling law holds for open-source** | r=0.702, +4.8 pts per 10× size |
| 9 | **Temporal progress: +0.53 pts/month overall** | r=0.503, significant only for closed-source |
| 10 | **Size + source_type + date explain 59.5% of model variance** | M2: adj-R²=0.553 |

---

## Generated Files

### Data tables
| File | Contents |
|------|----------|
| `desc_by_category.csv` | Mean/SD/min/max by category |
| `desc_by_round.csv` | Mean/SD/min/max by round |
| `desc_by_model.csv` | Per-model mean scores (ranked) |
| `anova_table.csv` | Type III ANOVA table |
| `tukey_category.csv` | Tukey HSD pairwise comparisons (category) |
| `tukey_round.csv` | Tukey HSD pairwise comparisons (round) |
| `corr_pearson.csv` | Full Pearson correlation matrix |
| `corr_spearman.csv` | Full Spearman correlation matrix |
| `pca_loadings.csv` | PC1–PC4 loadings for all metrics |
| `regression_coefs.csv` | OLS coefficients (CodeBLEU/ROUGE → J-LLM) |
| `interaction_cat_round.csv` | Category × Round mean scores + Δ |
| `model_category_means.csv` | All-model × all-category heatmap data |
| `multivariate_summary.csv` | High-level summary statistics |
| `open_closed_by_category.csv` | Open vs closed per category |
| `size_bin_stats.csv` | Performance by parameter size bin |
| `year_stats.csv` | Performance by release year |
| `extended_regression_summary.csv` | M1/M2/M3 regression comparison |

### Figures
| Figure | Contents |
|--------|----------|
| `fig1_correlation_heatmap` | Pearson + Spearman heatmaps for all 12 metrics |
| `fig2_pca` | Scree plot + biplot (PC1 vs PC2) |
| `fig3_interaction_cat_round` | Line plot + Δ bar chart for category × round |
| `fig4_anova_violin` | Score distributions by category and round |
| `fig5_regression` | Actual vs predicted + coefficient plot |
| `fig6_model_category_heatmap` | 35 models × 5 categories heatmap |
| `fig7_summary_dashboard` | 6-panel summary of sections 1–9 |
| `fig8_open_vs_closed` | Open vs closed: overall + by category + by round |
| `fig9_model_size_effect` | Size scatter (log scale) + size bin boxplot |
| `fig10_release_date_company` | Date scatter (open/closed) + company bar chart |
| `fig11_extended_dashboard` | 6-panel summary of sections 10–13 |
