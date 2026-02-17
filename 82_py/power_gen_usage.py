import math
import numpy as np
import matplotlib.pyplot as plt
from dataclasses import dataclass, asdict
from CoDR_equations import g, V_CRUISE
from ambiance import Atmosphere
from pint import UnitRegistry

ureg = UnitRegistry()  # keeps track of units


@dataclass
class AircraftConfig:
    """Input parameters for the DEP sizing model."""

    range_m: float = 2500e3 * ureg("m")
    V_cruise: float = V_CRUISE * ureg("m/s")
    alt_cruise_m: float = 3048.0 * ureg("m")

    Cd: float = 0.02

    # design vars
    mass_kg: float = 4000.0 * ureg("kg")
    wing_loading_kgm2: float = 180.0 * ureg("kg/m^2")

    # efficiencies & margin
    eta_prop: float = 0.85
    eta_motor: float = 0.95
    eta_inverter: float = 0.98
    eta_gearbox: float = 1.0
    eta_generator: float = 0.95
    power_margin: float = 1.15

    # Fuel/APU (Jet-A LHV ~42 MJ/kg)
    eta_apu_overall: float = 0.30
    LHV_MJ_per_kg: float = 42.0 * ureg("MJ/kg")


@dataclass
class MissionResults:
    """Calculated output metrics."""

    rho_cruise_kgm3: float = 0.0
    wing_area_m2: float = 0.0
    thrust_required_N: float = 0.0
    P_gen_elec_kW: float = 0.0
    P_gen_sized_kW: float = 0.0
    cruise_time_hr: float = 0.0
    cruise_energy_MWh: float = 0.0
    fuel_mass_cruise_kg: float = 0.0
    fuel_flow_cruise_kg_hr: float = 0.0
    gen_mass: float = 0.0


# Sizing model
class DEPSizingModel:
    def compute_performance(self, cfg: AircraftConfig) -> MissionResults:
        mission_cls = MissionResults()

        mission_cls.wing_area_m2 = (
            cfg.mass_kg / cfg.wing_loading_kgm2
        )  # wing area from mass wing loading

        # cruise drag -> thrust required
        mission_cls.rho_cruise_kgm3 = (
            Atmosphere(h=(cfg.alt_cruise_m.magnitude)).density[0]
        ) * ureg("kg/m^3")
        q = 0.5 * mission_cls.rho_cruise_kgm3 * (cfg.V_cruise) ** 2
        mission_cls.thrust_required_N = q * mission_cls.wing_area_m2 * cfg.Cd.magnitude

        # power calcs
        P_thrust = mission_cls.thrust_required_N * cfg.V_cruise
        P_shaft = P_thrust / (cfg.eta_prop)
        eta_down = cfg.eta_motor * cfg.eta_inverter * cfg.eta_gearbox
        P_bus = P_shaft / eta_down
        mission_cls.P_gen_elec_kW = (P_bus / cfg.eta_generator).to("kW")
        mission_cls.P_gen_sized_kW = mission_cls.P_gen_elec_kW * cfg.power_margin

        # mission time/energy/fuel
        mission_cls.cruise_time_hr = (cfg.range_m / cfg.V_cruise).to("hr")
        mission_cls.cruise_energy_MWh = (
            mission_cls.P_gen_elec_kW * (mission_cls.cruise_time_hr)
        ).to("MWh")

        E_MJ = mission_cls.cruise_energy_MWh.to("MJ")  # convert MWh back to MJ
        mission_cls.fuel_mass_cruise_kg = E_MJ / (
            cfg.eta_apu_overall * cfg.LHV_MJ_per_kg
        )

        mission_cls.fuel_flow_cruise_kg_hr = mission_cls.fuel_mass_cruise_kg / (
            mission_cls.cruise_time_hr
        )
        # assumes linear weight scaling from this: https://evtol.news/news/hard-core-hybrids
        mission_cls.gen_mass = (mission_cls.P_gen_sized_kW) * (
            295 * ureg("kg") / (370 * ureg("kW"))
        )
        return mission_cls


# Runner Script
if __name__ == "__main__":
    model = DEPSizingModel()
    config = AircraftConfig()

    final_res = model.compute_performance(config)

    print("=== DEP Generator Sizing + Takeoff + Cruise Fuel + Objective ===")
    print(f"Cruise density rho:          {final_res.rho_cruise_kgm3:.3f}")
    print(f"Wing area S:                {final_res.wing_area_m2:.2f}")
    print(f"Cruise thrust required:     {final_res.thrust_required_N:.0f}")
    print(f"Generator (cruise) output:  {final_res.P_gen_elec_kW:.1f}")
    print(f"Generator (sized) rating:   {final_res.P_gen_sized_kW:.1f}")
    print(f"Generator Mass:             {final_res.gen_mass:.1f}")
    print(f"Cruise time:                {final_res.cruise_time_hr:.2f}")
    print(f"Cruise energy:              {final_res.cruise_energy_MWh:.3f}")
    print(f"Cruise fuel mass:           {final_res.fuel_mass_cruise_kg:.1f}")
    print(f"Avg cruise fuel flow:       {final_res.fuel_flow_cruise_kg_hr:.1f}")

    # Sweep ranges
    wing_loading_vals = np.linspace(120, 300, 40)
    V_vals = np.linspace(90, 170, 40)
    TW_vals = np.linspace(0.2, 0.9, 40)

    plt.show()
