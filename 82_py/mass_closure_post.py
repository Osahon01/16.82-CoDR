"""Post processing script. Use to visualize optimizer results (it has already been tabulated)."""

import pickle
import numpy as np
from mass_closure_opt import FILE_NAME, MASS_TARGETS
import matplotlib.pyplot as plt

colors = plt.cm.viridis(np.linspace(0, 1, len(MASS_TARGETS)))

# open without re-running optimizer
with open(FILE_NAME, "rb") as f:
    data = pickle.load(f)

pareto_results_ar = data["pareto_results_ar"]
pareto_results_ws = data["pareto_results_ws"]

plt.figure()
for i, res in enumerate(pareto_results_ar):
    plt.scatter(
        res["AR"],
        res["v_cruise"],
        s=50,
        color=colors[i],
        label=f"{round(res['mass_target'], 2)} kg",
    )
plt.xlabel("AR", fontsize=16)
plt.ylabel("Cruise Velocity", fontsize=16)
plt.title("Cruise Velocity vs AR", fontsize=20)
plt.legend(fontsize=16)
plt.show()

plt.figure()
for i, res in enumerate(pareto_results_ws):
    plt.scatter(
        res["x_TO"],
        res["v_cruise"],
        s=50,
        color=colors[i],
        label=f"{round(res['mass_target'], 2)} kg",
    )
plt.xlabel("Takeoff Distance (m)", fontsize=16)
plt.ylabel("Cruise Velocity (m/s)", fontsize=16)
plt.title("Cruise Velocity vs Takeoff Distance", fontsize=20)
plt.legend(fontsize=16)
plt.show()
