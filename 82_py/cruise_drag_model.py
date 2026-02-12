import numpy as np
import matplotlib.pyplot as plt

# Constants
pi = np.pi
e = 0.8
CD0 = 0.032

# Cruise conditions
rho = 1       # density (kg/m^3) approximate
V = 96          # cruise speed (m/s)
S = 26           # wing area (m^2)

q = 0.5 * rho * V**2

# # Structural model
W_fixed = 3600 #30000      # N (everything except wing)
# k = 1000          # wing structural scaling constant

AR = np.linspace(3, 18, 200)
# W = W_fixed + k * AR**2 # Weight model
W = W_fixed
CL = W / (q * S) # Required CL
CDi = CL**2 / (pi * AR * e) # Induced drag
CD = CD0 + CDi # Total drag
LD = CL / CD # L/D

# Plot
plt.figure()
plt.plot(AR, LD)
plt.xlabel("Aspect Ratio")
plt.ylabel("L/D")
plt.title("L/D vs Aspect Ratio (Including Structural Penalty)")
plt.grid()
plt.show()

# Print optimum
best_index = np.argmax(LD)
print("Optimal AR:", AR[best_index])

