# Initial Prompt for ChatGPT

## Copy and paste this prompt to ChatGPT:

---

I need your help writing a LaTeX report for my EE571 Final Project: "State Estimation and Optimal Control of a 6-Mass Spring System."

**Report Requirements:**
- The report must be self-sufficient (the reader should not need to run code to understand results)
- Must include all necessary plots and figures
- Must explain solutions step-by-step
- Final output will be PDF format
- Use proper LaTeX formatting with equations, figures, tables, and references

**Project Overview:**

The project consists of 8 parts (Part 0 through Part 7):
- **Part 0**: Baseline Verification - Discretize model, verify open-loop behavior
- **Part 1**: Observability Analysis - Analyze which states are observable
- **Part 2**: Observer Design - Design Luenberger observer for state estimation
- **Part 3**: LQR Controller - Design optimal controller using estimated states
- **Part 4**: Reduced Input LQR - Redesign controller with only 2 inputs
- **Part 5**: Kalman Filter - Design optimal estimator for stochastic systems
- **Part 6**: LQG Controller - Combine LQR + Kalman filter
- **Part 7**: Sensor Augmentation - Analyze impact of adding more sensors

**System Description:**
- 6-mass spring chain (12 states: 6 positions + 6 velocities)
- 3 control inputs (u1, u2, u3)
- Discretized using Zero-Order Hold (ZOH) at Ts = 0.01s
- Cost function: J = sum(u'u + y1^2 + y6^2)

**How we will proceed:**
1. I will send you one part at a time with detailed documentation and code
2. You will write that section in LaTeX format
3. After completing each part, WAIT for my next input before continuing
4. Do NOT proceed to the next part until I send you the materials

**LaTeX Document Structure:**

Please start by creating the document preamble and title page. Use the following structure:

```latex
\documentclass[12pt,a4paper]{article}
\usepackage[utf8]{inputenc}
\usepackage{amsmath,amssymb,amsfonts}
\usepackage{graphicx}
\usepackage{float}
\usepackage{booktabs}
\usepackage{hyperref}
\usepackage{geometry}
\usepackage{caption}
\usepackage{subcaption}
\usepackage{listings}
\usepackage{xcolor}

\geometry{margin=1in}

\title{EE571 Final Project Report\\
State Estimation and Optimal Control of a 6-Mass Spring System}
\author{[Your Name]}
\date{\today}

\begin{document}
\maketitle
\tableofcontents
\newpage
```

Please create:
1. The LaTeX preamble (as shown above)
2. An Introduction section that:
   - Describes the 6-mass spring system
   - States the overall objectives
   - Outlines the 8 parts of the project
   - Introduces key concepts (state-space, discretization, LQR, Kalman filter)

After you complete this, STOP and wait for my next message with Part 0 materials.

---

## Attachments to include with this prompt:

Upload these files to ChatGPT:
1. `final/EE571_Final_Report.md` - Complete project summary
2. `final/anchor.md` - Project structure and guidelines

---

## Next Step:

After ChatGPT responds with the preamble and introduction, proceed to `01_part0_prompt.md`.
