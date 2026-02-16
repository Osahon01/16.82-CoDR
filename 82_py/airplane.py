# Imported libraries
import math
import numpy as np
from ambiance import Atmosphere
from power_gen_usage import ureg  # avoids instance issues

# Our models imported
from cruise_model import CruiseModel
from cruise_drag_model import parastic_drag
from power_gen_usage import AircraftConfig, DEPSizingModel
from climb_model import ClimbModel
from takeoff_model import TakeoffModel
from structural_wing_model import StructuralWingModel

# Design parameters (FIXED)
RANGE = 2500000 * ureg("m")
CLTO = 10  # Dalton will tell us
CDTO = 1  # Dalton
W = 12500 * 4.445  # N (converted from lbs)
S_W = 50  # kg/m^2
T_W = 0.3
h_cruise = 3048.0 * ureg("m")  # 10,000 ft in meters
gamma = math.radians(15.0)  # Climb angle in radians
eta_battery = 0.95
eta_generator = 0.92
eta_v_prop = 0.7
eta_add_prop = 0.7
epsilon_battery = 250.0 * 3600.0  # 250 Wh/kg converted to J/kg

# Cruise Model Constants
e = 0.8
CD0 = parastic_drag()  # Hard coded from cruise drag model for now, but will need to be part of a loop eventually

# Power Model
rho_cruise = Atmosphere(h=h_cruise).density

# Climb Model Constants
v_climb_vertical = 10.0  # m/s, vertical climb velocity (to be updated with more detailed model)
CD_climb = 0.03  # Assumed constant drag coefficient during climb (to be updated with more detailed model)

# Structural Model Constants
structural_FOS = 1.5  # Factor of safety for structural design (to be updated based on material choice and design requirements)
rho_spar = 1600.0  # kg/m^3, density of spar material (e.g., carbon fiber composite)
rho_skin = 1550.0  # kg/m^3, density of skin material (e.g., carbon fiber composite)
sigma_allow_design = 450e6  # Pa, allowable tensile/compressive stress for design (e.g., carbon fiber composite)
tau_allow_design = 80e6  # Pa, allowable shear stress for design (e.g., carbon fiber composite)

class Airplane:
    def __init__(self, v_cruise, AR):
        self.v_cruise = v_cruise * ureg("m/s")
        self.AR = AR
        self.S = W / S_W  # Wing area (m^2)
        self.T = W * T_W  # Takeoff thrust (N)

    def run_cruise_model(self):
        cruise = CruiseModel(
            s_ref=self.S,
            weight=W,
            v_cruise=self.v_cruise,
            h_cruise=h_cruise,
            AR=self.AR,
            e=e,
            Cd0=CD0,
        )
        drag = cruise.drag_total()
        CD_total = cruise.cd_total()
        return drag, CD_total

    def run_power_model(self):
        cfg = AircraftConfig(
            range_m=RANGE,
            V_cruise=self.v_cruise,
            alt_cruise_m=h_cruise,
            Cd=0.02,
            CLmax=7,
            mass_kg=4000 * ureg("kg"),
            wing_loading_kgm2=180 * ureg("kg/m^2"),
            thrust_loading_TW=0.4,
            eta_prop=0.85,
            eta_motor=0.95,
            eta_inverter=0.98,
            eta_gearbox=1,
            eta_generator=0.95,
            power_margin=1.15,
            eta_apu_overall=0.3,
            LHV_MJ_per_kg=42 * ureg("MJ/kg"),
        )

        power_cls = DEPSizingModel.compute_performance(cfg)

        return (
            power_cls.P_gen_elec_kW,
            power_cls.fuel_mass_cruise_kg,
            power_cls.cruise_time_hr,
            power_cls.gen_mass,
        )

    def run_climb_model(self, p_gen):
        climb = ClimbModel(
            C_D=CD_climb,
            S_W=S_W,
            gamma=gamma,
            P_generator=p_gen,
            eta_generator=eta_generator,
            eta_battery=eta_battery,
            epsilon_battery=epsilon_battery,
            h_cruise=h_cruise,
            S=self.S,
            v=v_climb_vertical
            )
        time_of_climb = climb.time_of_climb()
        m_battery = climb.get_m_battery()
        p_bat = climb.battery_power_required()
        return p_bat, m_battery, time_of_climb

    def run_takeoff_model(self, p_gen, p_bat):
        P_shaft_TO = p_gen * eta_generator + p_bat * eta_battery
        takeoff = TakeoffModel(self.T_W, S_W, W, P_shaft_TO, CLTO, CDTO)
        takeoff_distance = takeoff.takeoff_distance()
        return takeoff_distance
    
    def run_wing_structural_model(self, L_max):
        wing_structural_model = StructuralWingModel(
            L_max=L_max, 
            M_max=30_000.0,
            S=self.S,
            AR=self.AR,
            FOS=structural_FOS,
            # "Medium quality" carbon composite-ish densities TODO: Update with real material choices
            rho_spar=rho_spar,  # kg/m^3
            rho_skin=rho_skin,  # kg/m^3
            sigma_allow_design=sigma_allow_design,  # Pa
            tau_allow_design=tau_allow_design,  # Pa
        spar_mass = wing_structural_model.spar_mass()
        skin_mass = 0 # wing_structural_model.skin_mass()
        return spar_mass, skin_mass
)

    def runner(self):
        drag, CD_total = self.run_cruise_model()
        p_gen, m_gen, t_flight = self.run_power_model()
        p_bat, m_bat, t_climb = self.run_climb_model(p_gen)
        x_TO = self.run_takeoff_model(p_gen, p_bat)
        L_max = W
        spar_mass, skin_mass = self.run_wing_structural_model(L_max)
        masses = np.array([m_gen, m_bat, spar_mass, skin_mass])
        return x_TO, masses


drela_forehead = Airplane(v_cruise=80, AR=10)
# print(f'Cruise model test: {drela_forehead.run_cruise_model()}')
