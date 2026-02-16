# Imported libraries
import math
import numpy as np
from ambiance import Atmosphere

# Our models imported
from cruise_model import CruiseModel
from cruise_drag_model import parastic_drag
from power_gen_usage import AircraftConfig, MissionResults
from climb_model import ClimbModel
from takeoff_model import TakeoffModel

# Design parameters (FIXED)
CLTO = 10  # Dalton will tell us
CDTO = 1  # Dalton
W = 12500 * 4.445  # N (converted from lbs)
S_W = 50  # kg/m^2
T_W = 0.3
h_cruise = 3048.0  # 10,000 ft in meters
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


class Airplane:
    def __init__(self, v_cruise, AR):
        self.v_cruise = v_cruise
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
        power =MissionResults(rho_cruise_kgm3=rho_cruise, wing_area_m2=self.S, thrust_required_N=self.T)
        
        # power =MissionResults(rho_cruise_kgm3=rho_cruise, 
        #                       wing_area_m2=self.S, 
        #                       thrust_required_N=self.T, 
        #                       P_gen_elec_kW: float = 0, 
        #                       P_gen_sized_kW: float = 0, 
        #                       cruise_time_hr: float = 0, 
        #                       cruise_energy_MWh: float = 0, 
        #                       fuel_mass_cruise_kg: float = 0, 
        #                       fuel_flow_cruise_kg_hr: float = 0)
        
        MissionResults(
            rho_cruise,
            self.S,
            self.T,
            W,
            self.v_cruise,
            h_cruise,
            self.AR,
            e,
            CD0,
            eta_v_prop,
            eta_add_prop,
            epsilon_battery,
        )
        power.P_generator = power_required_for_cruise(self.v_cruise, self.AR)
        pass

    def run_climb_model(self, p_gen):
        climb = ClimbModel(
            T_W=self.T_W,
            C_D=CD0,  # TODO: update with actual CD from cruise model
            S_W=S_W,
            gamma=gamma,
            P_generator=p_gen,
            P_battery=0,  # This is deprecated @biruk 
            eta_generator=eta_generator,
            eta_battery=eta_battery,
            epsilon_battery=epsilon_battery,
            h_cruise=h_cruise,
        )
        time_of_climb = climb.time_of_climb()
        m_battery = climb.get_m_battery()
        return time_of_climb, m_battery

    def run_takeoff_model(self, p_gen, p_bat):
        P_shaft_TO = p_gen * eta_generator + p_bat * eta_battery
        takeoff = TakeoffModel(self.T_W, S_W, W, P_shaft_TO, CLTO, CDTO)
        takeoff_distance = takeoff.takeoff_distance()
        return takeoff_distance

    def runner(self):
        drag, CD_total = self.run_cruise_model()
        p_gen, m_gen, p_bat, m_bat, t_flight = self.run_power_model()
        p_bat = self.run_climb_model(p_gen) # To be updated
        x_TO = self.run_takeoff_model(p_gen, p_bat)
        # TODO: add structural model and get masses of components, then return all results in the masses array
        masses = np.array([m_gen, m_bat])
        return x_TO, masses

# Conrads climb --> # p_bat, # m_bat
# Zach's    
p_gen
m_gen
time of flight
Cruise fuel mass


drela_forehead = Airplane(v_cruise=80, AR=10)
# print(f'Cruise model test: {drela_forehead.run_cruise_model()}')
