import inspect
import re
from pathlib import Path

import sympy as sm
import sympy.physics.mechanics as me


def lagrangian_to_maptor_dynamics(lagranges_method, coordinates, control_forces, output_file=None):
    """
    Convert SymPy LagrangesMethod to MAPTOR dynamics format using universal mass matrix approach.

    Supports any mechanical system by using the fundamental equation q̈ = M⁻¹(f_passive + f_control)
    where M is the mass matrix, f_passive includes forces from forcelist, and f_control contains
    control forces/torques acting on generalized coordinates.

    Args:
        lagranges_method: SymPy LagrangesMethod object with passive forces in forcelist
        coordinates: List of generalized coordinates (e.g., [q1, q2])
        control_forces: SymPy Matrix of control forces/torques for each coordinate. Shape must be (len(coordinates), 1)
        output_file: Optional filename for saving generated dynamics in same directory as caller. If None, only prints to console.

    Returns:
        tuple: (casadi_equations, state_names) for programmatic use

    Examples:
        Cartpole with external force:

        >>> control_forces = sm.Matrix([F, 0])  # Force on x, none on theta
        >>> lagrangian_to_maptor_dynamics(LM, [x, theta], control_forces)

        Manipulator with joint torques:

        >>> control_forces = sm.Matrix([tau1, tau2])  # Torques on q1, q2
        >>> lagrangian_to_maptor_dynamics(LM, [q1, q2], control_forces)

        With backup file output:

        >>> lagrangian_to_maptor_dynamics(LM, coords, forces, "dynamics_backup.txt")

        Pure passive system (no control):

        >>> control_forces = sm.Matrix([0, 0])  # No control forces
        >>> lagrangian_to_maptor_dynamics(LM, [q1, q2], control_forces)
    """
    # Input validation
    if not hasattr(lagranges_method, "form_lagranges_equations"):
        raise ValueError("lagranges_method must be a SymPy LagrangesMethod object")
    if not coordinates:
        raise ValueError("coordinates list cannot be empty")
    if not hasattr(control_forces, "shape"):
        raise ValueError("control_forces must be a SymPy Matrix")

    n_coords = len(coordinates)
    if control_forces.shape != (n_coords, 1):
        raise ValueError(
            f"control_forces shape {control_forces.shape} must match coordinates length ({n_coords}, 1)"
        )

    # Build Lagrangian equations components
    try:
        # Form equations of motion to compute mass matrix and forcing
        lagranges_method.form_lagranges_equations()

        mass_matrix = lagranges_method.mass_matrix
        forcing = lagranges_method.forcing

        if mass_matrix.shape != (n_coords, n_coords):
            raise ValueError(
                f"Mass matrix shape {mass_matrix.shape} incompatible with {n_coords} coordinates"
            )
        if forcing.shape != (n_coords, 1):
            raise ValueError(
                f"Forcing vector shape {forcing.shape} incompatible with {n_coords} coordinates"
            )

    except Exception as e:
        raise ValueError(f"Failed to extract Lagrangian components: {e}") from e

    # Universal dynamics: q̈ = M⁻¹(f_passive + f_control)
    try:
        mass_matrix_inv = mass_matrix.inv()
        total_generalized_forces = forcing + control_forces
        explicit_accelerations = mass_matrix_inv * total_generalized_forces

        # Simplify each acceleration equation
        explicit_accelerations = [sm.simplify(expr) for expr in explicit_accelerations]

    except Exception as e:
        raise ValueError(f"Failed to compute dynamics - system may be singular: {e}") from e

    # Create first-order system: [q1_dot, q2_dot, ..., q1_ddot, q2_ddot, ...]
    first_derivatives = [coord.diff(me.dynamicsymbols._t) for coord in coordinates]
    first_order_system = first_derivatives + explicit_accelerations

    # Convert to CasADi syntax
    casadi_equations = _sympy_to_casadi_string(first_order_system)

    # Generate state names using same conversion logic
    coordinate_names = _sympy_to_casadi_string(coordinates)
    state_names = coordinate_names + [name + "_dot" for name in coordinate_names]

    # Extract control variable names from control_forces matrix
    control_names = []
    for i in range(n_coords):
        control_expr = control_forces[i]
        if control_expr == 0:
            continue  # Skip zero controls

        # Extract symbols from the control expression
        control_symbols = list(control_expr.free_symbols)
        if len(control_symbols) == 1:
            # Single symbol case (most common)
            control_name = str(control_symbols[0])
            control_names.append(control_name)
        elif len(control_symbols) == 0:
            # Constant non-zero control
            control_names.append(f"control_constant_{i}")
        else:
            # Multiple symbols or complex expression
            control_names.append(f"control_expr_{i}")

    # Generate output content
    output_content = _generate_output_content(state_names, control_names, casadi_equations)

    # Print copy-paste ready format (preserve existing behavior)
    print(output_content)

    # Optional file output in same directory as caller
    if output_file is not None:
        try:
            caller_frame = inspect.currentframe()
            if caller_frame is not None and caller_frame.f_back is not None:
                caller_filepath = caller_frame.f_back.f_code.co_filename
                caller_directory = Path(caller_filepath).parent
                output_path = caller_directory / output_file
            else:
                output_path = Path(output_file)
        except (AttributeError, OSError):
            output_path = Path(output_file)

        with open(output_path, "w") as f:
            f.write(output_content)
        print(f"\nDynamics saved to: {output_path}")

    return casadi_equations, state_names


def _generate_output_content(state_names, control_names, casadi_equations):
    """Generate formatted output content for console and file."""
    lines = [
        "CasADi MAPTOR Dynamics:",
        "=" * 60,
        "",
        "# State variables:",
    ]

    for name in state_names:
        lines.append(f"# {name} = phase.state('{name}')")

    lines.extend(
        [
            "",
            "# Control variables:",
        ]
    )

    for name in control_names:
        lines.append(f"# {name} = phase.control('{name}')")

    lines.extend(
        [
            "",
            "# MAPTOR dynamics dictionary:",
            "phase.dynamics({",
        ]
    )

    for name, eq_str in zip(state_names, casadi_equations, strict=False):
        lines.append(f"    {name}: {eq_str},")

    lines.append("})")

    return "\n".join(lines)


def _sympy_to_casadi_string(expressions):
    # Handle single expression case
    if not isinstance(expressions, list | tuple):
        expressions = [expressions]

    # Function mappings
    functions = {
        "atan2": "ca.atan2",
        "sqrt": "ca.sqrt",
        "sin": "ca.sin",
        "cos": "ca.cos",
        "tan": "ca.tan",
        "exp": "ca.exp",
        "log": "ca.log",
        "Abs": "ca.fabs",
        "asin": "ca.asin",
        "acos": "ca.acos",
        "atan": "ca.atan",
        "sinh": "ca.sinh",
        "cosh": "ca.cosh",
        "tanh": "ca.tanh",
    }

    # Derivative patterns - order matters!
    patterns = [
        # Handle second derivatives first
        (re.compile(r"Derivative\(([^,\(\)]+)\(t\),\s*\(t,\s*2\)\)"), r"\1_ddot"),
        # Handle first derivatives
        (re.compile(r"Derivative\(([^,\(\)]+)\(t\),\s*t\)"), r"\1_dot"),
        # Handle SymPy pretty printing shorthand
        (re.compile(r"\b([a-zA-Z][a-zA-Z0-9]*)ddot\b"), r"\1_ddot"),
        (re.compile(r"\b([a-zA-Z][a-zA-Z0-9]*)dot\b"), r"\1_dot"),
        # Remove (t) from base variables - MUST BE LAST
        (re.compile(r"\b([a-zA-Z][a-zA-Z0-9]*)\(t\)"), r"\1"),
    ]

    func_pattern = re.compile(r"\b(" + "|".join(re.escape(f) for f in functions.keys()) + r")\b")

    def _convert_single(expr):
        expr_str = str(expr)

        # Convert derivatives
        for pattern, replacement in patterns:
            expr_str = pattern.sub(replacement, expr_str)

        # Convert functions
        return func_pattern.sub(lambda m: functions[m.group(1)], expr_str)

    # Convert all expressions and handle Matrix format
    converted_expressions: list[str] = []
    for expr in expressions:
        converted = _convert_single(expr)

        # Post-process Matrix format if present
        if converted.startswith("Matrix([") and converted.endswith("])"):
            # Extract equations from Matrix([[eq1], [eq2]]) format
            inner = converted[9:-3]  # Remove 'Matrix([[' and ']])'
            converted_expressions.extend(eq.strip() for eq in inner.split("], ["))
        else:
            converted_expressions.append(converted)

    return converted_expressions
