import casadi as ca

from ..problem.state import PhaseDefinition


def _map_symbol_to_vector(symbol: ca.MX, vector: ca.MX, index: int, vector_length: int) -> ca.MX:
    return vector if vector_length == 1 else vector[index]


def _create_phase_symbol(base_name: str, phase_id: int, suffix: str = "") -> ca.MX:
    name = f"{base_name}_p{phase_id}{suffix}" if suffix else f"{base_name}_p{phase_id}"
    return ca.MX.sym(name, 1)  # type: ignore[arg-type]


def _create_state_symbols_for_phase(phase_id: int, num_states: int) -> tuple[ca.MX, ca.MX]:
    x0_vec = ca.vertcat(*[ca.MX.sym(f"x0_{i}_p{phase_id}", 1) for i in range(num_states)])  # type: ignore[arg-type]
    xf_vec = ca.vertcat(*[ca.MX.sym(f"xf_{i}_p{phase_id}", 1) for i in range(num_states)])  # type: ignore[arg-type]
    return ca.MX(x0_vec), ca.MX(xf_vec)


def _create_integral_symbol_for_phase(phase_id: int, num_integrals: int) -> ca.MX:
    if num_integrals > 0:
        return ca.vertcat(*[ca.MX.sym(f"q_{i}_p{phase_id}", 1) for i in range(num_integrals)])  # type: ignore[arg-type]
    return ca.MX.sym(f"q_p{phase_id}", 1)  # type: ignore[arg-type]


def _create_static_params_symbol(num_params: int) -> ca.MX:
    if num_params > 0:
        return ca.vertcat(*[ca.MX.sym(f"s_{i}", 1) for i in range(num_params)])  # type: ignore[arg-type]
    return ca.MX.sym("s", 1)  # type: ignore[arg-type]


def _build_unified_casadi_function_inputs(
    phase_def: PhaseDefinition,
    static_parameter_symbols: list[ca.MX] | None = None,
) -> tuple[ca.MX, ca.MX, ca.MX, ca.MX, list[ca.MX]]:
    state_syms = phase_def._get_ordered_state_symbols()
    control_syms = phase_def._get_ordered_control_symbols()

    states_vec = ca.vertcat(*state_syms) if state_syms else ca.MX()
    controls_vec = ca.vertcat(*control_syms) if control_syms else ca.MX()
    time = phase_def.sym_time if phase_def.sym_time is not None else ca.MX.sym("t", 1)  # type: ignore[arg-type]

    num_static_params = len(static_parameter_symbols) if static_parameter_symbols else 0
    static_params_vec = (
        ca.MX.sym("static_params", num_static_params, 1)  # type: ignore[arg-type]
        if num_static_params > 0
        else ca.MX.sym("static_params", 1, 1)  # type: ignore[arg-type]
    )

    function_inputs = [states_vec, controls_vec, time, static_params_vec]

    return (
        ca.MX(states_vec),
        ca.MX(controls_vec),
        ca.MX(time),
        ca.MX(static_params_vec),
        function_inputs,
    )


def _build_static_parameter_substitution_map(
    static_parameter_symbols: list[ca.MX] | None,
    static_params_vec: ca.MX,
) -> dict[ca.MX, ca.MX]:
    subs_map = {}
    num_static_params = len(static_parameter_symbols) if static_parameter_symbols else 0

    if static_parameter_symbols and num_static_params > 0:
        for i, param_sym in enumerate(static_parameter_symbols):
            subs_map[param_sym] = _map_symbol_to_vector(
                param_sym, static_params_vec, i, num_static_params
            )

    return subs_map


def _build_phase_inputs_for_phase(multiphase_state, phase_id: int) -> list[ca.MX]:
    phase_def = multiphase_state.phases[phase_id]

    t0 = (
        phase_def.sym_time_initial
        if phase_def.sym_time_initial is not None
        else _create_phase_symbol("t0", phase_id)
    )
    tf = (
        phase_def.sym_time_final
        if phase_def.sym_time_final is not None
        else _create_phase_symbol("tf", phase_id)
    )

    state_syms = phase_def._get_ordered_state_symbols()
    x0_vec, xf_vec = _create_state_symbols_for_phase(phase_id, len(state_syms))

    q_vec = _create_integral_symbol_for_phase(phase_id, phase_def.num_integrals)

    return [t0, tf, x0_vec, xf_vec, q_vec]


def _build_unified_multiphase_symbol_inputs(multiphase_state) -> tuple[list[ca.MX], ca.MX]:
    phase_inputs = []

    for phase_id in sorted(multiphase_state.phases.keys()):
        phase_inputs.extend(_build_phase_inputs_for_phase(multiphase_state, phase_id))

    static_param_syms = multiphase_state.static_parameters.get_ordered_parameter_symbols()
    s_vec = _create_static_params_symbol(len(static_param_syms))

    phase_inputs.append(s_vec)

    return phase_inputs, ca.MX(s_vec)


def _map_state_symbols_for_phase(
    phase_def: PhaseDefinition,
    x0_vec: ca.MX,
    xf_vec: ca.MX,
    phase_symbols_map: dict[ca.MX, ca.MX],
) -> None:
    state_syms = phase_def._get_ordered_state_symbols()
    state_initial_syms = phase_def._get_ordered_state_initial_symbols()
    state_final_syms = phase_def._get_ordered_state_final_symbols()
    num_states = len(state_syms)

    for i, (state_sym, initial_sym, final_sym) in enumerate(
        zip(state_syms, state_initial_syms, state_final_syms, strict=True)
    ):
        state_value = _map_symbol_to_vector(state_sym, xf_vec, i, num_states)
        initial_value = _map_symbol_to_vector(initial_sym, x0_vec, i, num_states)
        final_value = _map_symbol_to_vector(final_sym, xf_vec, i, num_states)

        phase_symbols_map[state_sym] = state_value
        phase_symbols_map[initial_sym] = initial_value
        phase_symbols_map[final_sym] = final_value


def _map_integral_symbols_for_phase(
    phase_def: PhaseDefinition, q_vec: ca.MX, phase_symbols_map: dict[ca.MX, ca.MX]
) -> None:
    for i, integral_sym in enumerate(phase_def.integral_symbols):
        phase_symbols_map[integral_sym] = _map_symbol_to_vector(
            integral_sym, q_vec, i, phase_def.num_integrals
        )


def _map_time_symbols_for_phase(
    t0: ca.MX, tf: ca.MX, phase_symbols_map: dict[ca.MX, ca.MX]
) -> None:
    phase_symbols_map[t0] = t0
    phase_symbols_map[tf] = tf


def _map_static_parameter_symbols(
    static_param_syms: list[ca.MX], s_vec: ca.MX, phase_symbols_map: dict[ca.MX, ca.MX]
) -> None:
    num_params = len(static_param_syms)
    for i, param_sym in enumerate(static_param_syms):
        phase_symbols_map[param_sym] = _map_symbol_to_vector(param_sym, s_vec, i, num_params)


def _build_unified_symbol_substitution_map(
    multiphase_state, phase_inputs: list[ca.MX], s_vec: ca.MX
) -> dict[ca.MX, ca.MX]:
    phase_symbols_map = {}
    input_idx = 0

    for phase_id in sorted(multiphase_state.phases.keys()):
        phase_def = multiphase_state.phases[phase_id]

        t0 = phase_inputs[input_idx]
        tf = phase_inputs[input_idx + 1]
        x0_vec = phase_inputs[input_idx + 2]
        xf_vec = phase_inputs[input_idx + 3]
        q_vec = phase_inputs[input_idx + 4]
        input_idx += 5

        _map_time_symbols_for_phase(t0, tf, phase_symbols_map)
        _map_state_symbols_for_phase(phase_def, x0_vec, xf_vec, phase_symbols_map)
        _map_integral_symbols_for_phase(phase_def, q_vec, phase_symbols_map)

    static_param_syms = multiphase_state.static_parameters.get_ordered_parameter_symbols()
    _map_static_parameter_symbols(static_param_syms, s_vec, phase_symbols_map)

    return phase_symbols_map
