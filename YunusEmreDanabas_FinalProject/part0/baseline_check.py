import numpy as np
import matplotlib.pyplot as plt
import os
from final.utils.model import build_continuous_model, discretize_zoh
from final.utils.simulation import simulate_discrete


def plot_signals(t, signals, labels, title, ylabel=None, save_path=None, grid=True):
    """Plot multiple signals with legend. Signals is 2D array where rows are different signals."""
    signals = np.atleast_2d(signals)
    num_signals, _ = signals.shape
    
    if ylabel is None:
        ylabel = 'Value'
    
    fig, ax = plt.subplots(figsize=(10, 6))
    
    for i in range(num_signals):
        ax.plot(t, signals[i, :], label=labels[i])
    
    ax.set_xlabel('Time (s)')
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.legend()
    if grid:
        ax.grid(True)
    
    if save_path is not None:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
    
    return fig, ax


def main():
    """Baseline verification: discretize and simulate open-loop system."""
    Ts = 0.01
    N = 1000
    x0 = np.array([0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0])
    
    A, B, C = build_continuous_model()
    Ad, Bd, Cd, Dd = discretize_zoh(A, B, C, Ts)
    
    u = np.zeros((3, N))
    x, y, t = simulate_discrete(Ad, Bd, Cd, x0, u, N, Ts=Ts)
    
    script_dir = os.path.dirname(os.path.abspath(__file__))
    output_dir = os.path.join(script_dir, 'outputs')
    os.makedirs(output_dir, exist_ok=True)
    
    plot_signals(t, y, ['y1'], 'Baseline System Output: y = Cx',
                 ylabel='Output', save_path=os.path.join(output_dir, 'output_plot.png'))
    
    plot_signals(t, x[:6, :], ['x1', 'x2', 'x3', 'x4', 'x5', 'x6'],
                 'All Mass Displacements: x_1..x_6',
                 ylabel='Displacement', save_path=os.path.join(output_dir, 'displacements_plot.png'))
    
    print(f"Part 0: Baseline verification complete")
    print(f"  Matrix dimensions: A {A.shape}, B {B.shape}, C {C.shape}")
    print(f"  Outputs saved to {output_dir}/")


if __name__ == '__main__':
    main()
