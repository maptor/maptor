NUMERICAL_ZERO = 1e-14
COORDINATE_PRECISION = 1e-12
TIME_PRECISION = 1e-10


LARGE_VALUE: float = 1e12  # General "infinity" replacement


# ODE solver constants
DEFAULT_ODE_RTOL = 1e-8
DEFAULT_ODE_ATOL_FACTOR = 1e-8
DEFAULT_ODE_METHOD: str = "RK45"
DEFAULT_ODE_MAX_STEP: float | None = None
DEFAULT_ERROR_SIM_POINTS: int = 50


# Adaptive algorithm defaults
DEFAULT_ADAPTIVE_ERROR_TOLERANCE: float = 1e-6
DEFAULT_ADAPTIVE_MAX_ITERATIONS: int = 30
DEFAULT_MIN_POLYNOMIAL_DEGREE: int = 3
DEFAULT_MAX_POLYNOMIAL_DEGREE: int = 8

# Plotting constants (aesthetic, no numerical impact)
DEFAULT_FIGURE_SIZE: tuple[float, float] = (12.0, 8.0)
DEFAULT_GRID_ALPHA: float = 0.3
DEFAULT_PHASE_BOUNDARY_ALPHA: float = 0.7
DEFAULT_PHASE_BOUNDARY_LINEWIDTH: float = 2.0
