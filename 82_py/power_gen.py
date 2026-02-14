"""
DEP sizing + takeoff metric + cruise fuel burn + parameter sweeps w/ contour plots.

Attribute-based version (no dictionary passing).
All math unchanged.
"""

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


# =========================
# DEP MODEL
# =========================
class DEPSizingModel:

    def __init__(
        self,
        range_m,
        V_cruise,
        alt_cruise_m,
        Cd,
        mass_kg,
        wing_loading_kgm2,
        thrust_loading_TW,
        CLmax,
        g,
        eta_prop,
        eta_motor,
        eta_inverter,
        eta_gearbox,
        eta_generator,
        power_margin,
        eta_apu_overall=0.30,
        LHV_MJ_per_kg=42.0,
        V_ref=125.0,
        w_to=1.0,
        w_V=1.0,
        wing_loading_limit=250.0,
        penalty_weight=50.0,
    ):

        # Store inputs
        self.range_m = range_m
        self.V_cruise = V_cruise
        self.alt_cruise_m = alt_cruise_m
        self.Cd = Cd
        self.mass_kg = mass_kg
        self.wing_loading_kgm2 = wing_loading_kgm2
        self.thrust_loading_TW = thrust_loading_TW
        self.CLmax = CLmax
        self.g = g

        self.eta_prop = eta_prop
        self.eta_motor = eta_motor
        self.eta_inverter = eta_inverter
        self.eta_gearbox = eta_gearbox
        self.eta_generator = eta_generator
        self.power_margin = power_margin

        self.eta_apu_overall = eta_apu_overall
        self.LHV_MJ_per_kg = LHV_MJ_per_kg

        self.V_ref = V_ref
        self.w_to = w_to
        self.w_V = w_V
        self.wing_loading_limit = wing_loading_limit
        self.penalty_weight = penalty_weight


    # -------------------------
    # Core Computation
    # -------------------------
    def compute(self):

        self.wing_area_m2 = self.mass_kg / self.wing_loading_kgm2

        self.rho_cruise_kgm3 = isa_density(self.alt_cruise_m)
        q = 0.5 * self.rho_cruise_kgm3 * self.V_cruise**2

        self.thrust_required_N = q * self.wing_area_m2 * self.Cd

        P_thrust = self.thrust_required_N * self.V_cruise
        P_shaft = P_thrust / self.eta_prop
        eta_down = self.eta_motor * self.eta_inverter * self.eta_gearbox
        P_bus = P_shaft / eta_down
        P_gen_elec = P_bus / self.eta_generator

        self.P_gen_elec_kW = P_gen_elec / 1000.0
        self.P_gen_sized_kW = self.P_gen_elec_kW * self.power_margin

        rho_to = isa_density(0.0)
        W_over_S = self.wing_loading_kgm2 * self.g

        self.x_to_metric = (
            (W_over_S / self.thrust_loading_TW)
            * (1.0 / (rho_to * self.g))
            * (1.0 / self.CLmax)
        )

        t_cruise = self.range_m / self.V_cruise
        self.cruise_time_hr = t_cruise / 3600.0

        E_cruise_J = P_gen_elec * t_cruise
        self.cruise_energy_MWh = E_cruise_J / 3.6e9

        E_MJ = self.cruise_energy_MWh * 3600.0
        self.fuel_mass_cruise_kg = (
            E_MJ / (self.eta_apu_overall * self.LHV_MJ_per_kg)
        )

        self.fuel_flow_cruise_kg_hr = (
            self.fuel_mass_cruise_kg / self.cruise_time_hr
        )


    # -------------------------
    # Objective
    # -------------------------
    def compute_objective(self, x_to_ref):

        x_to_hat = self.x_to_metric / x_to_ref
        V_hat = self.V_cruise / self.V_ref

        if self.wing_loading_kgm2 <= self.wing_loading_limit:
            penalty = 0.0
        else:
            penalty = (
                self.penalty_weight
                * ((self.wing_loading_kgm2 - self.wing_loading_limit)
                   / self.wing_loading_limit) ** 2
            )

        self.objective_J = (
            self.w_to * x_to_hat - self.w_V * V_hat + penalty
        )


# =========================
# SWEEP + CONTOUR
# =========================
def sweep_2d(
    base_params,
    x_key,
    x_vals,
    y_key,
    y_vals,
    z_attr,
):

    X, Y = np.meshgrid(x_vals, y_vals)
    Z = np.full_like(X, np.nan, dtype=float)

    base_model = DEPSizingModel(**base_params)
    base_model.compute()
    x_to_ref = base_model.x_to_metric

    for i in range(Y.shape[0]):
        for j in range(X.shape[1]):

            p = dict(base_params)
            p[x_key] = float(X[i, j])
            p[y_key] = float(Y[i, j])

            model = DEPSizingModel(**p)
            model.compute()
            model.compute_objective(x_to_ref)

            Z[i, j] = getattr(model, z_attr)

    return X, Y, Z


def contour_plot(
    X,
    Y,
    Z,
    x_label,
    y_label,
    title,
    levels=20,
):

    plt.figure()
    cs = plt.contourf(X, Y, Z, levels=levels)
    plt.colorbar(cs, label=title)
    plt.xlabel(x_label)
    plt.ylabel(y_label)
    plt.title(title)
    plt.tight_layout()
    plt.show()


# =========================
# Example Run
# =========================
if __name__ == "__main__":

    params = dict(
        range_m=2500e3,
        V_cruise=125.0,
        alt_cruise_m=10000.0 * 0.3048,
        Cd=0.02,
        mass_kg=4000,
        wing_loading_kgm2=180,
        thrust_loading_TW=0.4,
        CLmax=7.0,
        g=9.80665,
        eta_prop=0.85,
        eta_motor=0.95,
        eta_inverter=0.98,
        eta_gearbox=1.0,
        eta_generator=0.95,
        power_margin=1.15,
    )

    model = DEPSizingModel(**params)
    model.compute()
    model.compute_objective(model.x_to_metric)

    print("Generator Power (kW):", model.P_gen_elec_kW)
    print("Takeoff metric:", model.x_to_metric)
    print("Fuel mass (kg):", model.fuel_mass_cruise_kg)

    wing_loading_vals = np.linspace(120, 300, 40)
    V_vals = np.linspace(90, 170, 40)

    X, Y, Z = sweep_2d(
        params,
        "wing_loading_kgm2",
        wing_loading_vals,
        "V_cruise",
        V_vals,
        "objective_J",
    )

    contour_plot(
        X,
        Y,
        Z,
        "Wing loading (kg/m^2)",
        "Cruise speed (m/s)",
        "Objective J",
    )
