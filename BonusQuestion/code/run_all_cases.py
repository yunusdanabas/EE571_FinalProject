#!/usr/bin/env python3
"""
run_all_cases.py - Run 6-case experiment matrix for vehicle tracking

Experiment matrix:
- 2 regulators: LQR and Pole Placement
- 3 initial error scales: 1x, 2x, 3x
Total: 6 runs
"""

import numpy as np

# Define 6 run configurations
configs = [
    {'regulator': 'LQR', 'scale': 1},
    {'regulator': 'LQR', 'scale': 2},
    {'regulator': 'LQR', 'scale': 3},
    {'regulator': 'PP',  'scale': 1},
    {'regulator': 'PP',  'scale': 2},
    {'regulator': 'PP',  'scale': 3},
]

def main():
    print("Experiment Matrix:")
    print("==================")
    for i, cfg in enumerate(configs):
        print(f"Run {i+1}: {cfg['regulator']} regulator, scale {cfg['scale']}x")
    print()
    
    # TODO: Import vehicle_tracking module
    # TODO: For each configuration:
    #   1. Set regulator type (LQR or PP)
    #   2. Set initial condition scale (1, 2, or 3)
    #   3. Run simulation
    #   4. Collect results (trajectory, errors, inputs)
    #   5. Compute metrics
    #   6. Generate plots
    #   7. Save plots to ../results/plots/
    #   8. Save metrics to ../results/logs/
    
    print("NOTE: Controller implementation not yet complete.")
    print("This skeleton defines the experiment matrix only.")

if __name__ == "__main__":
    main()
