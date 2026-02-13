from takeoff_model import TakeoffModel
import matplotlib.pyplot as plt
import numpy as np

# Old code for plotting x_TO versus cruise velocity
"""# Fixed parameters
S = 25.9  # m^2
W = 3600 * 9.81 # N
CLTO = 1.4
CDTO = 0.08
CD0 = 0.024  # Parasitic drag coefficient (Cessna Caravan)
AR = 9.3  # Aspect ratio (Cessna Caravan)
e = 0.85  # Oswald efficiency (Cessna Caravan)

# Define parameter ranges
power_values = np.linspace(200000, 1550000, 10)  # several different power values (W)
sw_values = np.linspace(50, 150, 12)   # several different wing loadings (kg/m^2)
tw = 0.3

# Plot setup
fig, ax = plt.subplots(figsize=(10, 7))
colors = plt.cm.tab10(np.linspace(0, 1, len(power_values)))

# Plot lines for each power value
for idx, P in enumerate(power_values):
    takeoff_distances = []
    cruise_velocities = []
    tw_labels = []
    
    for sw in sw_values:
        W_S = sw
        T_W_takeoff = tw
        P_shaft_TO = P
        plane = Takeoff_model(T_W_takeoff, W_S, W, P_shaft_TO, CLTO, CDTO, CD0, AR, e)
        
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
cessna_caravan = Takeoff_model(T_W_takeoff=0.3, W_S=50, W=3600*9.81, P_shaft_TO=503000, CLTO=1.4, CDTO=0.08, CD0=0.024, AR=8.3, e=0.85)
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
plt.show()"""
