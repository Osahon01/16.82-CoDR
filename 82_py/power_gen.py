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
from dataclasses import dataclass, asdict
from CoDR_equations import g, V_CRUISE
from ambiance import Atmosphere


@dataclass
class AircraftConfig:
    """Input parameters for the DEP sizing model."""

    # TODO: update inputs
    # NOTE: try to avoid redefining vars for consistency

    range_m: float = 2500e3
    V_cruise: float = V_CRUISE
    alt_cruise_m: float = 10000.0 * 0.3048

    Cd: float = 0.02
    CLmax: float = 7.0

    # design vars
    mass_kg: float = (
        4000.0  # NOTE: would prefer ureg implementation. See CoDr for example
    )
    wing_loading_kgm2: float = 180.0
    thrust_loading_TW: float = 0.4

    # efficiencies & margin
    eta_prop: float = 0.85
    eta_motor: float = 0.95
    eta_inverter: float = 0.98
    eta_gearbox: float = 1.0
    eta_generator: float = 0.95
    power_margin: float = 1.15

    # Fuel/APU (Jet-A LHV ~42 MJ/kg)
    eta_apu_overall: float = 0.30
    LHV_MJ_per_kg: float = 42.0

    rho_to_kgm3: float = Atmosphere(h=0).density[0]  # Sea level standard
    V_ref: float = V_CRUISE
    w_to: float = 1.0
    w_V: float = 1.0
    wing_loading_limit: float = 250.0
    penalty_weight: float = 50.0


@dataclass
class MissionResults:
    """Calculated output metrics."""

    rho_cruise_kgm3: float = 0.0
    wing_area_m2: float = 0.0
    thrust_required_N: float = 0.0
    P_gen_elec_kW: float = 0.0
    P_gen_sized_kW: float = 0.0
    x_to_metric: float = 0.0
    cruise_time_hr: float = 0.0
    cruise_energy_MWh: float = 0.0
    fuel_mass_cruise_kg: float = 0.0
    fuel_flow_cruise_kg_hr: float = 0.0
    objective_J: float = 0.0


# Sizing model
class DEPSizingModel:
    def isa_density(self, alt_m: float) -> float:
        # TODO: would prefer using ureg and ambiance class to avoid errors; update hard coded numbers to take in altitude (h)
        R, T0, p0, L = 287.05287, 288.15, 101325.0, 0.0065
        alt_m = max(0.0, alt_m)
        if alt_m > 11000.0:
            raise ValueError("isa_density only implemented up to 11,000 m")
        T = T0 - L * alt_m
        p = p0 * (T / T0) ** (g / (R * L))
        return p / (R * T)

    def compute_performance(
        self, cfg: AircraftConfig, x_to_ref: float = 1.0
    ) -> MissionResults:
        res = MissionResults()

        res.wing_area_m2 = (
            cfg.mass_kg / cfg.wing_loading_kgm2
        )  # wing area from mass wing loading

        # cruise drag -> thrust required
        res.rho_cruise_kgm3 = self.isa_density(cfg.alt_cruise_m)
        q = 0.5 * res.rho_cruise_kgm3 * cfg.V_cruise**2
        res.thrust_required_N = q * res.wing_area_m2 * cfg.Cd

        # power calcs
        P_thrust = res.thrust_required_N * cfg.V_cruise
        P_shaft = P_thrust / max(cfg.eta_prop, 1e-12)
        eta_down = max(cfg.eta_motor * cfg.eta_inverter * cfg.eta_gearbox, 1e-12)
        P_bus = P_shaft / eta_down
        res.P_gen_elec_kW = (P_bus / max(cfg.eta_generator, 1e-12)) / 1000.0
        res.P_gen_sized_kW = res.P_gen_elec_kW * cfg.power_margin

        # takeoff metric
        W_over_S = cfg.wing_loading_kgm2 * g
        res.x_to_metric = (
            (W_over_S / max(cfg.thrust_loading_TW, 1e-12))
            * (1.0 / (cfg.rho_to_kgm3 * g))
            * (1.0 / max(cfg.CLmax, 1e-12))
        )

        # mission time/energy/fuel
        res.cruise_time_hr = (cfg.range_m / max(cfg.V_cruise, 1e-12)) / 3600.0
        res.cruise_energy_MWh = (
            res.P_gen_elec_kW * 1000.0 * (res.cruise_time_hr * 3600.0)
        ) / 3.6e9

        E_MJ = res.cruise_energy_MWh * 3.6e9 / 1e6  # convert MWh back to MJ
        res.fuel_mass_cruise_kg = E_MJ / max(
            cfg.eta_apu_overall * cfg.LHV_MJ_per_kg, 1e-12
        )
        res.fuel_flow_cruise_kg_hr = res.fuel_mass_cruise_kg / max(
            res.cruise_time_hr, 1e-12
        )

        # obj fn
        x_to_hat = res.x_to_metric / max(x_to_ref, 1e-12)
        V_hat = cfg.V_cruise / max(cfg.V_ref, 1e-12)

        penalty = 0.0
        if cfg.wing_loading_kgm2 > cfg.wing_loading_limit:
            penalty = (
                cfg.penalty_weight
                * (
                    (cfg.wing_loading_kgm2 - cfg.wing_loading_limit)
                    / cfg.wing_loading_limit
                )
                ** 2
            )

        res.objective_J = cfg.w_to * x_to_hat - cfg.w_V * V_hat + penalty

        return res

    def sweep_2d(
        self,
        cfg_base: AircraftConfig,
        x_attr: str,
        x_vals: np.ndarray,
        y_attr: str,
        y_vals: np.ndarray,
        z_attr: str,
    ):
        X, Y = np.meshgrid(x_vals, y_vals)
        Z = np.zeros_like(X)

        # baseline
        base_res = self.compute_performance(cfg_base)
        x_to_ref = base_res.x_to_metric

        for i in range(len(y_vals)):
            for j in range(len(x_vals)):
                # Clone config and update the two sweep attributes
                point_cfg = AircraftConfig(**asdict(cfg_base))
                setattr(point_cfg, x_attr, X[i, j])
                setattr(point_cfg, y_attr, Y[i, j])

                res = self.compute_performance(point_cfg, x_to_ref=x_to_ref)
                Z[i, j] = getattr(res, z_attr)
        return X, Y, Z

    def contour_plot(
        self, X, Y, Z, x_label, y_label, title, levels=25, limit_line=None
    ):
        plt.figure(figsize=(9, 6))
        cp = plt.contourf(X, Y, Z, levels=levels, cmap="viridis")
        plt.colorbar(cp, label=title)
        plt.xlabel(x_label)
        plt.ylabel(y_label)
        plt.title(title)

        if limit_line:
            plt.axvline(
                limit_line, color="red", linestyle="--", label="Wing Loading Limit"
            )
            plt.legend()
        plt.tight_layout()


# Runner Script
if __name__ == "__main__":
    model = DEPSizingModel()
    config = AircraftConfig()

    # baseline calc
    # ran it twice - one to get the baseline x_to_ref, and once to get the normalized objective
    temp_res = model.compute_performance(config)
    final_res = model.compute_performance(config, x_to_ref=temp_res.x_to_metric)

    print("=== DEP Generator Sizing + Takeoff + Cruise Fuel + Objective ===")
    print(f"Cruise density rho:          {final_res.rho_cruise_kgm3:.3f} kg/m^3")
    print(f"Wing area S:                {final_res.wing_area_m2:.2f} m^2")
    print(f"Cruise thrust required:     {final_res.thrust_required_N:.0f} N")
    print(f"Generator (cruise) output:  {final_res.P_gen_elec_kW:.1f} kW")
    print(f"Generator (sized) rating:   {final_res.P_gen_sized_kW:.1f} kW")
    print(f"x_to metric:                {final_res.x_to_metric:.4f}")
    print(f"Cruise time:                {final_res.cruise_time_hr:.2f} hr")
    print(f"Cruise energy:              {final_res.cruise_energy_MWh:.3f} MWh")
    print(f"Cruise fuel mass:           {final_res.fuel_mass_cruise_kg:.1f} kg")
    print(f"Avg cruise fuel flow:       {final_res.fuel_flow_cruise_kg_hr:.1f} kg/hr")
    print(f"Objective J (lower=better): {final_res.objective_J:.3f}")

    # Sweep ranges
    wing_loading_vals = np.linspace(120, 300, 40)
    V_vals = np.linspace(90, 170, 40)
    TW_vals = np.linspace(0.2, 0.9, 40)

    # Sweep A: Objective J
    XA, YA, ZA = model.sweep_2d(
        config,
        "wing_loading_kgm2",
        wing_loading_vals,
        "V_cruise",
        V_vals,
        "objective_J",
    )
    model.contour_plot(
        XA,
        YA,
        ZA,
        "Wing Loading (kg/m^2)",
        "Cruise Speed (m/s)",
        "Objective J (min x_to, max V, penalize m/S>250)",
        limit_line=config.wing_loading_limit,
    )

    # Sweep B: Required Gen Power
    XB, YB, ZB = model.sweep_2d(
        config,
        "wing_loading_kgm2",
        wing_loading_vals,
        "V_cruise",
        V_vals,
        "P_gen_elec_kW",
    )
    model.contour_plot(
        XB,
        YB,
        ZB,
        "Wing Loading (kg/m^2)",
        "Cruise Speed (m/s)",
        "Required generator electrical power at cruise (kW)",
        limit_line=config.wing_loading_limit,
    )

    # Sweep C: Takeoff Metric
    XC, YC, ZC = model.sweep_2d(
        config,
        "wing_loading_kgm2",
        wing_loading_vals,
        "thrust_loading_TW",
        TW_vals,
        "x_to_metric",
    )
    model.contour_plot(
        XC,
        YC,
        ZC,
        "Wing Loading (kg/m^2)",
        "Thrust Loading (T/W)",
        "Takeoff metric x_to",
        limit_line=config.wing_loading_limit,
    )

    plt.show()
