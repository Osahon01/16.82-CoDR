import numpy as np
from airplane import Airplane
import matplotlib.pyplot as plt
from scipy.optimize import minimize

MASS_TARGETS = np.linspace(4500, 8000, 5)
V_SWEEP = np.linspace(70, 130, 5)
AR_SWEEP = np.linspace(8, 15, 5)

pareto_results = []
bounds = [(70, 130), (8, 15)]

for mass_target in MASS_TARGETS:
    v_opt_list = []
    AR_opt_list = []
    x_TO_list = []
    mass_error_list = []

    for v0 in V_SWEEP:
        for ar0 in AR_SWEEP:
            x0 = [v0, ar0]

            # minimize mass error
            def obj_mass_error(x):
                v, ar = x
                plane = Airplane(v, ar)
                _, masses = plane.runner()
                return abs(sum(masses) - mass_target)

            res = minimize(
                obj_mass_error,
                x0,
                method="L-BFGS-B",
                bounds=bounds,
                options={"ftol": 1e-3, "gtol": 1e-3, "maxfun": 50},
            )

            v_best, AR_best = res.x
            plane = Airplane(v_best, AR_best)
            x_TO, masses = plane.runner()
            mass_error = abs(sum(masses) - mass_target)

            v_opt_list.append(v_best)
            AR_opt_list.append(AR_best)
            x_TO_list.append(x_TO)
            mass_error_list.append(mass_error)

    v_opt_list = np.array(v_opt_list)
    AR_opt_list = np.array(AR_opt_list)
    x_TO_list = np.array(x_TO_list)
    mass_error_list = np.array(mass_error_list)

    feasible_mask = mass_error_list < 50  # kg tolerance

    pareto_results.append(
        {
            "mass_target": mass_target,
            "v_cruise": v_opt_list[feasible_mask],
            "AR": AR_opt_list[feasible_mask],
            "x_TO": x_TO_list[feasible_mask],
            "mass_error": mass_error_list[feasible_mask],
        }
    )

plt.figure(figsize=(8, 6))
colors = plt.cm.viridis(np.linspace(0, 1, len(MASS_TARGETS)))

for i, res in enumerate(pareto_results):
    plt.scatter(
        res["v_cruise"],
        res["AR"],
        s=50,
        color=colors[i],
        label=f"{(res['mass_target'])} kg",
    )

plt.xlabel("Cruise Velocity (m/s)")
plt.ylabel("Aspect Ratio (AR)")
plt.title("Pareto-like Front: v_cruise vs AR for each Mass Target")
plt.legend()
plt.grid(True)
plt.show()

plt.figure(figsize=(8, 6))
for i, res in enumerate(pareto_results):
    plt.scatter(
        res["v_cruise"],
        res["x_TO"],
        s=50,
        color=colors[i],
        label=f"{res['mass_target']} kg",
    )

plt.xlabel("Cruise Velocity (m/s)")
plt.ylabel("Takeoff Distance x_TO (m)")
plt.title("Takeoff Distance vs Cruise Velocity (feasible designs)")
plt.legend()
plt.grid(True)
plt.show()
