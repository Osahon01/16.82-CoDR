"""
DEP sizing + takeoff metric + cruise fuel burn + parameter sweeps w/ contour plots.

Adds:
- Cruise fuel mass (kg) and fuel flow (kg/s, kg/hr) using APU efficiency + Jet-A LHV
- Objective function to: minimize takeoff metric, maximize cruise speed, enforce wing loading <= 250 kg/m^2
- Extra contour: required generator electrical power at cruise (kW) (this is already P_gen_elec_kW)

Notes:
- LHV should be ~42 MJ/kg (not kJ/kg).
- eta_apu_overall is "fuel -> electrical bus" overall efficiency.
"""

import math
import numpy as np
import matplotlib.pyplot as plt


# -------------------------
# Atmosphere (ISA, up to 11 km)
# -------------------------
def isa_density(alt_m: float) -> float:
    g0 = 9.80665
    R = 287.05287
    T0 = 288.15
    p0 = 101325.0
    L = 0.0065

    alt_m = max(0.0, alt_m)
    if alt_m > 11000.0:
        raise ValueError("isa_density only implemented up to 11,000 m")

    T = T0 - L * alt_m
    p = p0 * (T / T0) ** (g0 / (R * L))
    return p / (R * T)


# -------------------------
# Core calculations
# -------------------------
def generator_kW_and_xto(params: dict) -> dict:
    # Mission
    R_m = float(params["range_m"])
    V = float(params["V_cruise"])
    alt_m = float(params["alt_cruise_m"])

    # Aero
    Cd = float(params["Cd"])

    # Aircraft/design variables
    m_kg = float(params["mass_kg"])
    g = float(params["g"])
    m_over_S = float(params["wing_loading_kgm2"])  # kg/m^2
    T_over_W = float(params["thrust_loading_TW"])  # -

    # Takeoff
    CLmax = float(params["CLmax"])
    rho_to = params.get("rho_to_kgm3", None)
    if rho_to is None:
        rho_to = isa_density(0.0)

    # Efficiencies + margin
    eta_prop = float(params["eta_prop"])
    eta_motor = float(params["eta_motor"])
    eta_inv = float(params["eta_inverter"])
    eta_gb = float(params["eta_gearbox"])
    eta_gen = float(params["eta_generator"])
    margin = float(params["power_margin"])

    # 1) Wing area from mass wing loading
    S = m_kg / m_over_S  # m^2

    # 2) Cruise drag -> thrust required
    rho_cr = isa_density(alt_m)
    q = 0.5 * rho_cr * V**2
    D = q * S * Cd
    T_req = D

    # 3) Power chain to generator electrical output
    P_thrust = T_req * V  # W
    P_shaft = P_thrust / max(eta_prop, 1e-12)
    eta_down = max(eta_motor * eta_inv * eta_gb, 1e-12)
    P_bus = P_shaft / eta_down
    P_gen_elec = P_bus / max(eta_gen, 1e-12)  # W (continuous cruise electrical)
    P_gen_sized = P_gen_elec * margin  # W (nameplate sizing)

    # 4) Takeoff roll metric (your form)
    # x_to = (W/S)/(T/W) * (1/(rho*g))*(1/CLmax)
    W_over_S = m_over_S * g  # N/m^2
    x_to = (
        (W_over_S / max(T_over_W, 1e-12))
        * (1.0 / (rho_to * g))
        * (1.0 / max(CLmax, 1e-12))
    )

    # 5) Mission time/energy
    t_cruise = R_m / max(V, 1e-12)  # s
    E_cruise_J = P_gen_elec * t_cruise  # J
    E_cruise_MWh = E_cruise_J / 3.6e9  # (J -> MWh)

    return {
        "rho_cruise_kgm3": rho_cr,
        "wing_area_m2": S,
        "thrust_required_N": T_req,
        "P_gen_elec_kW": P_gen_elec
        / 1000.0,  # REQUIRED generator electrical power at cruise
        "P_gen_sized_kW": P_gen_sized / 1000.0,
        "x_to_metric": x_to,
        "cruise_time_hr": t_cruise / 3600.0,
        "cruise_energy_MWh": E_cruise_MWh,
    }


# -------------------------
# Fuel burn from cruise electrical energy
# -------------------------
def cruise_fuel_from_energy(
    E_elec_MWh: float, eta_apu: float, LHV_MJ_per_kg: float = 42.0
) -> float:
    E_MJ = E_elec_MWh * 3600.0
    return E_MJ / max(eta_apu * LHV_MJ_per_kg, 1e-12)


def add_cruise_fuel_metrics(out: dict, params: dict) -> dict:
    eta_apu = float(params.get("eta_apu_overall", 0.30))  # fuel -> electrical overall
    LHV = float(params.get("LHV_MJ_per_kg", 42.0))

    fuel_kg = cruise_fuel_from_energy(
        out["cruise_energy_MWh"], eta_apu=eta_apu, LHV_MJ_per_kg=LHV
    )
    t_s = out["cruise_time_hr"] * 3600.0

    out2 = dict(out)
    out2["fuel_mass_cruise_kg"] = fuel_kg
    out2["fuel_flow_cruise_kg_s"] = fuel_kg / max(t_s, 1e-12)
    out2["fuel_flow_cruise_kg_hr"] = out2["fuel_flow_cruise_kg_s"] * 3600.0
    return out2


# -------------------------
# Objective function (min x_to, max V, wing loading <= 250)
# -------------------------
def objective_function(
    out: dict,
    params: dict,
    x_to_ref: float,
    V_ref: float = 125.0,
    w_to: float = 1.0,
    w_V: float = 1.0,
    wing_loading_limit: float = 250.0,
    penalty_weight: float = 50.0,
) -> float:
    """
    Lower is better:
      J = w_to*(x_to/x_to_ref) - w_V*(V/V_ref) + penalty(wing_loading)

    penalty is 0 if wing_loading <= limit, else quadratic.
    """
    x_to_hat = out["x_to_metric"] / max(x_to_ref, 1e-12)
    V_hat = float(params["V_cruise"]) / max(V_ref, 1e-12)

    m_over_S = float(params["wing_loading_kgm2"])
    if m_over_S <= wing_loading_limit:
        penalty = 0.0
    else:
        penalty = (
            penalty_weight * ((m_over_S - wing_loading_limit) / wing_loading_limit) ** 2
        )

    return w_to * x_to_hat - w_V * V_hat + penalty


def add_objective_metric(
    out: dict,
    params: dict,
    x_to_ref: float,
    V_ref: float = 125.0,
    w_to: float = 1.0,
    w_V: float = 1.0,
    wing_loading_limit: float = 250.0,
    penalty_weight: float = 50.0,
) -> dict:
    out2 = dict(out)
    out2["objective_J"] = objective_function(
        out,
        params,
        x_to_ref=x_to_ref,
        V_ref=V_ref,
        w_to=w_to,
        w_V=w_V,
        wing_loading_limit=wing_loading_limit,
        penalty_weight=penalty_weight,
    )
    return out2


# -------------------------
# Generic 2D sweep + contour plot
# -------------------------
def sweep_2d(
    params_base: dict,
    x_key: str,
    x_vals: np.ndarray,
    y_key: str,
    y_vals: np.ndarray,
    z_key: str,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    X, Y = np.meshgrid(x_vals, y_vals)  # shape (len(y), len(x))
    Z = np.full_like(X, np.nan, dtype=float)

    # Baseline reference for normalization (objective)
    base_out = add_cruise_fuel_metrics(generator_kW_and_xto(params_base), params_base)
    x_to_ref = base_out["x_to_metric"]

    for i in range(Y.shape[0]):
        for j in range(X.shape[1]):
            p = dict(params_base)
            p[x_key] = float(X[i, j])
            p[y_key] = float(Y[i, j])

            out = generator_kW_and_xto(p)
            out = add_cruise_fuel_metrics(out, p)
            out = add_objective_metric(
                out,
                p,
                x_to_ref=x_to_ref,
                V_ref=params_base.get("V_ref", 125.0),
                w_to=params_base.get("w_to", 1.0),
                w_V=params_base.get("w_V", 1.0),
                wing_loading_limit=params_base.get("wing_loading_limit", 250.0),
                penalty_weight=params_base.get("penalty_weight", 50.0),
            )

            if z_key not in out:
                raise KeyError(
                    f"z_key='{z_key}' not found. Available: {list(out.keys())}"
                )

            Z[i, j] = float(out[z_key])

    return X, Y, Z


def contour_plot(
    X: np.ndarray,
    Y: np.ndarray,
    Z: np.ndarray,
    x_label: str,
    y_label: str,
    title: str,
    levels: int = 20,
    add_limit_line: dict | None = None,
):
    """
    add_limit_line (optional): {"orientation":"v"|"h", "value":..., "label":...}
    """
    plt.figure()
    cs = plt.contourf(X, Y, Z, levels=levels)
    plt.colorbar(cs, label=title)
    plt.xlabel(x_label)
    plt.ylabel(y_label)
    plt.title(title)

    if add_limit_line is not None:
        if add_limit_line["orientation"] == "v":
            plt.axvline(add_limit_line["value"], linestyle="--")
            if add_limit_line.get("label"):
                plt.text(
                    add_limit_line["value"],
                    np.nanmin(Y),
                    add_limit_line["label"],
                    rotation=90,
                    va="bottom",
                )
        elif add_limit_line["orientation"] == "h":
            plt.axhline(add_limit_line["value"], linestyle="--")
            if add_limit_line.get("label"):
                plt.text(
                    np.nanmin(X),
                    add_limit_line["value"],
                    add_limit_line["label"],
                    va="bottom",
                )
    plt.tight_layout()
    plt.show()


# -------------------------
# Example run
# -------------------------
if __name__ == "__main__":
    params = {
        # Given mission
        "range_m": 2500e3,
        "V_cruise": 125.0,
        "alt_cruise_m": 10000.0 * 0.3048,
        # Aero assumption
        "Cd": 0.02,  # can update this with better drag assumption from the takeoff model
        # Aircraft/design variables
        "mass_kg": 4000,
        "wing_loading_kgm2": 180,  # kg/m^2
        "thrust_loading_TW": 0.4,  # -
        # Takeoff
        "CLmax": 7.0,
        # Constants
        "g": 9.80665,
        # Efficiencies + margin
        "eta_prop": 0.85,
        "eta_motor": 0.95,
        "eta_inverter": 0.98,
        "eta_gearbox": 1.0,
        "eta_generator": 0.95,
        "power_margin": 1.15,  # margin from design power
        # Fuel/APU conversion
        "eta_apu_overall": 0.30,  # fuel -> electrical overall efficiency
        "LHV_MJ_per_kg": 42.0,  # Jet-A LHV (MJ/kg)
        # Objective tuning (optional overrides)
        "V_ref": 125.0,
        "w_to": 1.0,
        "w_V": 1.0,
        "wing_loading_limit": 250.0,
        "penalty_weight": 50.0,
    }

    base_out = add_cruise_fuel_metrics(generator_kW_and_xto(params), params)
    x_to_ref = base_out["x_to_metric"]
    out = add_objective_metric(
        base_out,
        params,
        x_to_ref=x_to_ref,
        V_ref=params["V_ref"],
        w_to=params["w_to"],
        w_V=params["w_V"],
        wing_loading_limit=params["wing_loading_limit"],
        penalty_weight=params["penalty_weight"],
    )

    print("=== DEP Generator Sizing + Takeoff + Cruise Fuel + Objective ===")
    print(f"Cruise density rho:          {out['rho_cruise_kgm3']:.3f} kg/m^3")
    print(f"Wing area S:                {out['wing_area_m2']:.2f} m^2")
    print(f"Cruise thrust required:     {out['thrust_required_N']:.0f} N")
    print(f"Generator (cruise) output:  {out['P_gen_elec_kW']:.1f} kW")
    print(f"Generator (sized) rating:   {out['P_gen_sized_kW']:.1f} kW")
    print(f"x_to metric:                {out['x_to_metric']:.4f}")
    print(f"Cruise time:                {out['cruise_time_hr']:.2f} hr")
    print(f"Cruise energy:              {out['cruise_energy_MWh']:.3f} MWh")
    print(f"Cruise fuel mass:           {out['fuel_mass_cruise_kg']:.1f} kg")
    print(f"Avg cruise fuel flow:       {out['fuel_flow_cruise_kg_hr']:.1f} kg/hr")
    print(f"Objective J (lower=better): {out['objective_J']:.3f}")

    # -------------------------
    # Sweep A: wing loading vs cruise speed -> objective (includes wing loading <= 250 penalty)
    # -------------------------
    wing_loading_vals = np.linspace(120, 300, 40)  # kg/m^2 (includes infeasible region)
    V_vals = np.linspace(90, 170, 40)  # m/s

    XA, YA, ZA = sweep_2d(
        params_base=params,
        x_key="wing_loading_kgm2",
        x_vals=wing_loading_vals,
        y_key="V_cruise",
        y_vals=V_vals,
        z_key="objective_J",
    )

    contour_plot(
        XA,
        YA,
        ZA,
        x_label="Wing loading m/S (kg/m^2)",
        y_label="Cruise speed V (m/s)",
        title="Objective J (min x_to, max V, penalize m/S>250)",
        levels=25,
        add_limit_line={
            "orientation": "v",
            "value": params["wing_loading_limit"],
            "label": "m/S limit",
        },
    )

    # -------------------------
    # Sweep B: wing loading vs cruise speed -> REQUIRED generator electrical power at cruise (kW)
    # -------------------------
    XB, YB, ZB = sweep_2d(
        params_base=params,
        x_key="wing_loading_kgm2",
        x_vals=wing_loading_vals,
        y_key="V_cruise",
        y_vals=V_vals,
        z_key="P_gen_elec_kW",  # <-- required cruise generator power consumption
    )

    contour_plot(
        XB,
        YB,
        ZB,
        x_label="Wing loading m/S (kg/m^2)",
        y_label="Cruise speed V (m/s)",
        title="Required generator electrical power at cruise (kW)",
        levels=25,
        add_limit_line={
            "orientation": "v",
            "value": params["wing_loading_limit"],
            "label": "m/S limit",
        },
    )

    # -------------------------
    # (Optional) Sweep C: wing loading vs T/W -> x_to metric (to visualize takeoff driver)
    # -------------------------
    TW_vals = np.linspace(0.2, 0.9, 40)

    XC, YC, ZC = sweep_2d(
        params_base=params,
        x_key="wing_loading_kgm2",
        x_vals=wing_loading_vals,
        y_key="thrust_loading_TW",
        y_vals=TW_vals,
        z_key="x_to_metric",
    )

    contour_plot(
        XC,
        YC,
        ZC,
        x_label="Wing loading m/S (kg/m^2)",
        y_label="T/W (-)",
        title="Takeoff metric x_to (your formula)",
        levels=25,
        add_limit_line={
            "orientation": "v",
            "value": params["wing_loading_limit"],
            "label": "m/S limit",
        },
    )
