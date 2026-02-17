import numpy as np
from scipy.integrate import solve_ivp
from scipy.optimize import fsolve
from CoDR_equations import g
from ambiance import Atmosphere


# Define the Takeoff model class
class TakeoffModel:
    def __init__(self, T_W_takeoff, W_S, W, P_shaft_TO, CLTO, CDTO):  # , CD0, AR, e):
        # Inputs (knowns)
        self.T_W_takeoff = T_W_takeoff
        self.W_S = W_S
        self.W = W
        self.P_shaft_TO = P_shaft_TO
        self.CLTO = CLTO
        self.CDTO = CDTO
        # self.CD0 = CD0  # Parasitic drag coefficient
        # self.AR = AR  # Aspect ratio
        # self.e = e  # Oswald efficiency factor

        # Constants
        self.g = g  # Acceleration due to gravity (m/s^2)
        self.rho_SL = Atmosphere(h=0).density  # Air density at sea level (kg/m^3)
        self.rho_cruise = None  # Atmosphere(h=H_END).density  # Air density at cruise (kg/m^3) - CURRENTLY ASSUMING 3000m CRUISE

        self.eta_v = 0.7  # Propeller efficiency (assumed constant for simplicity)
        self.eta_add = 0.7  # Additional efficiency factor from swirl and propwash at low speeds (assumed constant for simplicity)
        self.v0 = 15.0  # Lower bound for thrust calculation to prevent unrealistic values at very low cruise speeds

        # Derived Quantities
        self.S = self.W / (self.W_S * g)  # Wing area (m^2)
        self.T_takeoff = self.W * self.T_W_takeoff  # Takeoff thrust (N)

        # Placeholders
        self.v_TO = 0.0  # Placeholder for takeoff velocity, will be calculated based on CLTO and other parameters
        self.x_TO = 0.0  # (m) Placeholder for takeoff rolling distance

    def get_T_cruise(self, v):
        # Calculate cruise thrust using propeller power and efficiency
        v = max(v, self.v0)  # Ensure velocity is above the lower bound
        T_prop = (self.P * self.eta_v) / (v)
        return T_prop

    def get_A_prop(self, v):
        # Calculate cruise thrust using propeller power and efficiency
        A_prop = (
            self.T**3
            / (self.P_shaft_TO**2 * self.rho_SL)
            * 1
            / (self.eta_v * self.eta_add)
        )
        return A_prop

    def get_SW(self):
        # Calculate wing loading in kg/m^2
        return self.W / self.S

    def get_TW_takeoff(self):
        # Return thrust-to-weight ratio at takeoff
        return self.T_W

    def takeoff_distance(self):
        # Differential equation for takeoff distance
        def equations(t, y):
            x, x_dot = y
            d2x_dt2 = self.g * (
                self.T_W_takeoff
                - 0.5 * self.rho_SL * self.S / self.W * self.CDTO * x_dot**2
            )
            return [x_dot, d2x_dt2]

        # Initial conditions
        y0 = [0, 0]  # Initial position and velocity
        t_span = (0, 100)  # Time span for the simulation
        sol = solve_ivp(equations, t_span, y0, dense_output=True)

        # Find x when L >= W
        for t in np.linspace(0, 100, 4000):
            x = sol.sol(t)[0]
            v = sol.sol(t)[1]
            L = 0.5 * self.rho_SL * v**2 * self.S * self.CLTO
            if L >= self.W:
                self.v_TO = v  # Store takeoff velocity for later use
                return x
        return None

    def get_CDi(self, CL):
        # Calculate induced drag coefficient
        CDi = (CL**2) / (np.pi * self.AR * self.e)
        return CDi

    # Cruise model to be implemented elsewhere
    """def cruise_speed(self):
        
        def equations(vars):
            v_cruise, CL = vars
            eq1 = CL - (self.W * self.g) / (0.5 * self.rho_cruise * v_cruise**2 * self.S)
            eq2 = v_cruise - ((self.P * self.eta_v) / (0.5 * self.rho_cruise * self.S * (self.CD0 + self.get_CDi(CL))))**(1/3)
            return [eq1, eq2]
        
        # Initial guess
        v_guess = 50  # m/s
        CL_guess = 0.5
        
        solution = fsolve(equations, [v_guess, CL_guess])
        v_cruise = solution[0]
        return v_cruise
    
    def range(self):
        # Simplified range calculation based on cruise speed and endurance
        # This would need additional parameters like fuel weight for accurate calculation
        return None  # Placeholder"""


# Create an instance of the airplane class - Cessna Numbers
# R_prop_cessna = 1.32  # Propeller radius for Cessna Caravan (m)
# A_prop_cessna = np.pi * R_prop_cessna**2  # Propeller area for Cessna Caravan (m^2)
# cessna_caravan = takeoff_model(T_W_takeoff=0.3, W_S=50, W=3600*9.81, P_shaft_TO=503000, CLTO=1.4, CDTO=0.08, CD0=0.024, AR=8.3, e=0.85)
# print(cessna_caravan.takeoff_distance())
