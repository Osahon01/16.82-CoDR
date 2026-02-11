import numpy as np
from scipy.integrate import solve_ivp
from scipy.optimize import fsolve

# Constants
g = 9.81  # Acceleration due to gravity (m/s^2)
rho = 1.225  # Air density at sea level (kg/m^3)

# Define the airplane class
class Airplane:
    def __init__(self, S, W, P, CLTO, CDTO, A_prop, CD0=0.024, AR=9.3, e=0.85):
        self.S = S
        self.W = W
        self.P = P
        self.CLTO = CLTO
        self.CDTO = CDTO
        self.A_prop = A_prop
        self.CD0 = CD0  # Parasitic drag coefficient
        self.AR = AR  # Aspect ratio
        self.e = e  # Oswald efficiency factor
        self.eta_v = 0.7  # Propeller efficiency (assumed constant for simplicity)
        self.eta_add = 0.7  # Additional efficiency factor from swirl and propwash at low speeds (assumed constant for simplicity)
        self.v0 = 15.0 # Lower bound for thrust calculation to prevent unrealistic values at very low cruise speeds
        self.v_TO = 0.0 # Placeholder for takeoff velocity, will be calculated based on CLTO and other parameters
        self.T_takeoff = 0 # Placeholder for takeoff thrust, will be calculated based on power and efficiency
    
    def get_T_cruise(self, v):
        # Calculate cruise thrust using propeller power and efficiency
        v = max(v, self.v0)  # Ensure velocity is above the lower bound
        T_prop = (self.P * self.eta_v) / (v)
        return T_prop
    
    def get_T_takeoff(self, v):
        # Calculate cruise thrust using propeller power and efficiency
        T_prop = ((self.P * self.eta_v * self.eta_add * np.sqrt(self.A_prop * rho)))**(2/3)
        return T_prop
    
    def get_SW(self):
        # Calculate wing loading in kg/m^2
        return self.W / self.S
    
    def get_TW_takeoff(self):
        # Calculate thrust-to-weight ratio
        return self.get_T_takeoff(self.v_TO) / (self.W)
    
    def takeoff_distance(self):
        # Differential equation for takeoff distance
        def equations(t, y):
            x, x_dot = y
            d2x_dt2 = g * (self.get_T_takeoff(x_dot) / self.W - 0.5 * rho * self.S / self.W * self.CDTO * x_dot**2)
            return [x_dot, d2x_dt2]

        # Initial conditions
        y0 = [0, 0]  # Initial position and velocity
        t_span = (0, 100)  # Time span for the simulation
        sol = solve_ivp(equations, t_span, y0, dense_output=True)

        # Find x when L >= W
        for t in np.linspace(0, 100, 4000):
            x = sol.sol(t)[0]
            v = sol.sol(t)[1]
            L = 0.5  * rho * v**2 * self.S * self.CLTO
            if L >= self.W:
                self.v_TO = v  # Store takeoff velocity for later use
                return x
        return None
    def get_CDi(self, CL):
        # Calculate induced drag coefficient
        CDi = (CL**2) / (np.pi * self.AR * self.e)
        return CDi
    
    def cruise_speed(self):
        
        def equations(vars):
            v_cruise, CL = vars
            eq1 = CL - (self.W * g) / (0.5 * rho * v_cruise**2 * self.S)
            eq2 = v_cruise - ((self.P * self.eta_v) / (0.5 * rho * self.S * (self.CD0 + self.get_CDi(CL))))**(1/3)
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
        return None  # Placeholder


# Create an instance of the airplane class
R_prop_cessna = 1.32  # Propeller radius for Cessna Caravan (m)
A_prop_cessna = np.pi * R_prop_cessna**2  # Propeller area for Cessna Caravan (m^2)
cessna_caravan = Airplane(S=25.9, W=3629*g, P=503000, CLTO=1.4, CDTO=0.08, A_prop=A_prop_cessna, CD0=0.024, AR=9.3, e=0.85)
print(cessna_caravan.takeoff_distance(), cessna_caravan.cruise_speed())

# --- Line Plot Section ---
import matplotlib.pyplot as plt

# Fixed parameters
S = 25.9  # m^2
CLTO = 1.4
CDTO = 0.08
CD0 = 0.024  # Parasitic drag coefficient (Cessna Caravan)
AR = 9.3  # Aspect ratio (Cessna Caravan)
e = 0.85  # Oswald efficiency (Cessna Caravan)
A_prop = A_prop_cessna  # Use the same propeller area as the Cessna Caravan for consistency

# Define parameter ranges
power_values = np.linspace(200000, 1550000, 10)  # several different power values (W)
sw_values = np.linspace(50, 150, 12)   # several different wing loadings (kg/m^2)

# Plot setup
fig, ax = plt.subplots(figsize=(10, 7))
colors = plt.cm.tab10(np.linspace(0, 1, len(power_values)))

# Plot lines for each power value
for idx, P in enumerate(power_values):
    takeoff_distances = []
    cruise_velocities = []
    tw_labels = []
    
    for sw in sw_values:
        W = sw * S * g
        plane = Airplane(S, W, P, CLTO, CDTO, A_prop, CD0=CD0, AR=AR, e=e)
        
        # Calculate takeoff distance and cruise velocity
        d = plane.takeoff_distance()
        v = plane.cruise_speed()
        tw = plane.get_TW_takeoff()
        
        # Only add valid points
        if d is not None:
            takeoff_distances.append(d)
            cruise_velocities.append(v)
            tw_labels.append(tw)
    
    # Plot line for this power value using average T/W for label
    if tw_labels:
        avg_tw = np.mean(tw_labels)
        ax.plot(takeoff_distances, cruise_velocities, marker='o', linestyle='-', 
                color=colors[idx], label=f'P = {P/1e6:.2f}M W (T/W avg = {avg_tw:.3f})', linewidth=2, markersize=6)

ax.set_xlabel('Takeoff Distance (m)', fontsize=12)
ax.set_ylabel('Cruise Velocity (m/s)', fontsize=12)
ax.set_title(f'Cruise Velocity vs Takeoff Distance (CLTO={CLTO}, CDTO={CDTO}, CD0={CD0}, AR={AR}, e={e})', fontsize=13)

# Plot Cessna Caravan as a red star
cessna_d = cessna_caravan.takeoff_distance()
cessna_v = cessna_caravan.cruise_speed()
if cessna_d is not None:
    ax.plot(cessna_d, cessna_v, marker='*', markersize=20, color='red', label='Cessna Caravan', linestyle='none', zorder=5)

# Add labels for wing loading regions
# Low wing loading (low S/W, high altitude, more points to the left)
ax.text(0, 0.95, 'low wing loading', transform=ax.transAxes, fontsize=11, 
        verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))

# High wing loading (high S/W, points to the right)
ax.text(0.75, 0.25, 'high wing loading', transform=ax.transAxes, fontsize=11, 
        verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))

ax.legend(fontsize=10)
ax.grid(True, alpha=0.3)
plt.tight_layout()
plt.show()