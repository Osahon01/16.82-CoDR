# Imported libraries
import math
import numpy as np
from ambiance import Atmosphere

# Our models imported
from cruise_model import CruiseModel
from cruise_drag_model import parastic_drag
from power_gen_usage import (
    ureg,
    AircraftConfig,
    DEPSizingModel,
)  # avoids instance issues
from climb_model import ClimbModel
from takeoff_model import TakeoffModel
from structural_wing_model import StructuralWingModel
from CoDR_equations import g

# Design parameters (FIXED)
N_pass = 9
RANGE = 2500000 * ureg("m")
xTO = 300 / 3.28084  # Takeoff distance in meters (converted from 300 ft)
CLTO = 6.1  # Dalton will tell us
CDTO = 1.59  # Dalton
CMTO = 1.3  # Dalton (?!!)
W = 12500 * 4.445  # N (converted from lbs)
W_S = 130  # kg/m^2
T_W = 0.3
A_prop_over_S = (
    0.05  # Propeller disk area as a fraction of wing area (assumed for sizing purposes)
)
h_cruise = 3048.0 * ureg("m")  # 10,000 ft in meters
gamma = math.radians(25.0)  # Climb angle in radians
eta_battery = 0.95
eta_generator = 0.92
eta_v_prop = 0.7
eta_add_prop = 0.7
epsilon_battery = 250.0 * 3600.0  # 250 Wh/kg converted to J/kg

# Cruise Model Constants
e = 0.8
CD0 = parastic_drag()  # Hard coded from cruise drag model for now, but will need to be part of a loop eventually

# Power Model
rho_cruise = Atmosphere(h=h_cruise.magnitude).density[0]  # in SI units

# Climb Model Constants
v_climb_vertical = (
    25.0  # m/s, vertical climb velocity (to be updated with more detailed model)
)
CD_climb = 0.1  # Assumed constant drag coefficient during climb (to be updated with more detailed model)

# Structural Model Constants
structural_FOS = 1.5  # Factor of safety for structural design (to be updated based on material choice and design requirements)
rho_spar = 1600.0  # kg/m^3, density of spar material (e.g., carbon fiber composite)
rho_skin = 1550.0  # kg/m^3, density of skin material (e.g., carbon fiber composite)
sigma_allow_design = 450e6  # Pa, allowable tensile/compressive stress for design (e.g., carbon fiber composite)
tau_allow_design = (
    80e6  # Pa, allowable shear stress for design (e.g., carbon fiber composite)
)


class Airplane:
    def __init__(
        self, v_cruise, AR, W, xTO=(300.0 / 3.28), W_S=W_S, A_prop_over_S=A_prop_over_S
    ):
        self.v_cruise = v_cruise * ureg("m/s")
        self.AR = AR
        self.W = W
        self.xTO = xTO
        self.W_S = W_S
        self.A_prop_over_S = A_prop_over_S
        self.S = (self.W / self.W_S) / g  # Wing area (m^2)
        self.A_prop = self.A_prop_over_S * self.S  # Propeller disk area (m^2)

    def run_cruise_model(self):
        cruise = CruiseModel(
            s_ref=self.S,
            weight=self.W,
            v_cruise=self.v_cruise,
            h_cruise=h_cruise,
            AR=self.AR,
            e=e,
            Cd0=CD0,
        )
        drag = cruise.drag_total()
        CD_total = cruise.cd_total()
        return drag.to("N"), CD_total

    def run_power_model(self, CD_total):
        cfg = AircraftConfig(
            range_m=RANGE,
            V_cruise=self.v_cruise,
            alt_cruise_m=h_cruise,
            Cd=CD_total,
            mass_kg=(self.W / g) * ureg("kg"),
            wing_loading_kgm2=self.W_S * ureg("kg/m^2"),
            eta_prop=eta_v_prop * eta_add_prop,
            eta_motor=0.95,
            eta_inverter=0.98,
            eta_gearbox=1,
            eta_generator=0.95,
            power_margin=1.15,
            eta_apu_overall=0.3,
            LHV_MJ_per_kg=42 * ureg("MJ/kg"),
        )

        power_cls = DEPSizingModel().compute_performance(cfg)

        return (
            power_cls.P_gen_elec_kW,
            power_cls.fuel_mass_cruise_kg,
            power_cls.cruise_time_hr,
            power_cls.gen_mass,
        )

    def run_takeoff_model(self, P_gen):
        takeoff = TakeoffModel(
            W_S=self.W_S,
            W=self.W,
            P_gen=P_gen.to("W").magnitude,
            x_TO_target=self.xTO,
            CLTO=CLTO,
            CDTO=CDTO,
            CMTO=CMTO,
            AR=self.AR,
            A_prop=self.A_prop,
        )
        p_bat_takeoff = takeoff.solve_for_battery_power()
        takeoff_torsion = takeoff.get_torsion_moment()
        m_batt_takeoff = takeoff.get_battery_mass_takeoff()
        takeoff.TW_takeoff = takeoff.get_TW_takeoff()
        return takeoff_torsion, m_batt_takeoff

    def run_climb_model(self, p_gen):
        climb = ClimbModel(
            C_D=CD_climb,
            W_S=self.W_S,
            gamma=gamma,
            P_generator=p_gen,
            eta_generator=eta_generator,
            eta_battery=eta_battery,
            epsilon_battery=epsilon_battery,
            h_cruise=h_cruise,
            S=self.S,
            v_climb_vertical=v_climb_vertical,
            W=self.W,
        )  # pyright: ignore[reportCallIssue]
        time_of_climb = climb.time_of_climb()
        m_battery = climb.get_m_battery()
        p_bat = climb.battery_power_required()
        return p_bat, m_battery, time_of_climb

    def run_wing_structural_model(self, L_max, M_Max):
        wing_structural_model = StructuralWingModel(
            L_max=L_max,
            M_max=M_Max,
            S=self.S,
            AR=self.AR,
            FOS=structural_FOS,
            # "Medium quality" carbon composite-ish densities TODO: Update with real material choices
            rho_spar=rho_spar,  # kg/m^3
            rho_skin=rho_skin,  # kg/m^3
            sigma_allow_design=sigma_allow_design,  # Pa
            tau_allow_design=tau_allow_design,  # Pa
        )
        spar_mass = wing_structural_model.spar_mass()
        skin_mass = wing_structural_model.skin_mass()
        return spar_mass, skin_mass

    def get_passenger_mass(self):
        # Assuming an average passenger mass of 100 kg (including luggage)
        return N_pass * 100 * ureg("kg")

    def runner(self):
        drag, CD_total = self.run_cruise_model()
        p_gen, m_fuel, t_flight, m_gen = self.run_power_model(CD_total)
        M_max, m_batt_takeoff = self.run_takeoff_model(p_gen)
        p_bat, m_bat_climb, t_climb = self.run_climb_model(p_gen)
        L_max = self.W
        spar_mass, skin_mass = self.run_wing_structural_model(L_max, M_max)
        masses = np.array(
            [
                m_gen.magnitude,
                max(m_batt_takeoff, m_bat_climb),
                m_fuel.magnitude,
                spar_mass,
                float(skin_mass),
                self.get_passenger_mass().magnitude,
            ]
        )
        return masses


drela_forehead = Airplane(v_cruise=70, AR=8, W=W, xTO=xTO, W_S=W_S)
masses = drela_forehead.runner()
drag, CD_total = drela_forehead.run_cruise_model()
print(
    f"{50 * '='}\nCruise model test\nDrag: {round(drag, 2)}\nCD_total: {(round(CD_total, 2))}"
    f"\nmasses: {masses}\n{50 * '='}"
)

"""drela_forehead_2 = Airplane(v_cruise=70, AR=15, W=W)
x_TO, masses = drela_forehead_2.runner()
drag, CD_total = drela_forehead_2.run_cruise_model()
print(
    f"{50 * '='}\nCruise model test\nDrag: {round(drag, 2)}\nCD_total: {(round(CD_total, 2))}"
    f"\nx_T0: {round(x_TO, 2)}\nmasses: {round(masses[0], 2)}\n{50 * '='}"
)

drela_forehead_2 = Airplane(v_cruise=130, AR=8, W=W)
x_TO, masses = drela_forehead_2.runner()
drag, CD_total = drela_forehead_2.run_cruise_model()
print(
    f"{50 * '='}\nCruise model test\nDrag: {round(drag, 2)}\nCD_total: {(round(CD_total, 2))}"
    f"\nx_T0: {round(x_TO, 2)}\nmasses: {round(masses[0], 2)}\n{50 * '='}"
)

drela_forehead_2 = Airplane(v_cruise=130, AR=15, W=W)
x_TO, masses = drela_forehead_2.runner()
drag, CD_total = drela_forehead_2.run_cruise_model()
print(
    f"{50 * '='}\nCruise model test\nDrag: {round(drag, 2)}\nCD_total: {(round(CD_total, 2))}"
    f"\nx_T0: {round(x_TO, 2)}\nmasses: {round(masses[0], 2)}\n{50 * '='}"
)"""
