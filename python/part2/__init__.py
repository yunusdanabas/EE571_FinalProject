"""
Part 2: Observer Design with Augmented Sensor Matrix.
"""

from .observer_design import (
    get_part2_C_matrix,
    design_observer,
    design_observer_pole_placement,
    compute_observability_rank
)

__all__ = [
    'get_part2_C_matrix',
    'design_observer',
    'design_observer_pole_placement',
    'compute_observability_rank'
]

