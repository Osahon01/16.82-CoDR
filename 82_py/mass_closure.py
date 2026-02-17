import numpy as np
from airplane import Airplane
import matplotlib.pyplot as plt
from scipy.optimize import minimize
from CoDR_equations import g

MASS_TARGETS = np.linspace(4500, 8000, 2)
V_SWEEP = np.linspace(50, 130, 2)


# for fixed run 1
AR_SWEEP = np.linspace(4, 15, 2)
W_S_FIXED = 130

# for fixed run 2
AR_FIXED = 9
W_S_SWEEP = np.linspace(50, 90, 2)

pareto_results_ar = []
pareto_results_ws = []
bounds1 = [(50, 130), (4, 15)]  # for ar and vcruise sweep
bounds2 = [(50, 130), (50, 90)]  # for w/s and vcruise sweep


# minimize mass error
def obj_mass_error1(x):
    v, ar = x
    plane = Airplane(v, AR=ar, W=weight, W_S=W_S_FIXED)
    _, masses = plane.runner()
    return abs(sum(masses) - mass_target)


def obj_mass_error2(x):
    v, w_s = x
    plane = Airplane(v, AR=AR_FIXED, W=weight, W_S=w_s)
    _, masses = plane.runner()
    return abs(sum(masses) - mass_target)


def optimizer_loop(
    v=False,
    ar=False,
    weight=0.0,
    v_opt_list=[],
    AR_opt_list=[],
    x_TO_list=[],
    mass_error_list=[],
    w_s=W_S_SWEEP,
):
    if not ar:
        for ws0 in W_S_SWEEP:
            x0 = [v0, ws0]
            res = minimize(
                obj_mass_error2,
                x0,
                method="L-BFGS-B",
                bounds=bounds2,
                options={"ftol": 1e-1, "gtol": 1e-1, "maxfun": 25},
            )

            v_best, AR_best = res.x
            plane = Airplane(v_best, AR_best, W=weight)
            x_TO, masses = plane.runner()
            mass_error = abs(sum(masses) - mass_target)

            v_opt_list.append(v_best)
            AR_opt_list.append(AR_best)
            x_TO_list.append(x_TO)
            mass_error_list.append(mass_error)

        return v_opt_list, AR_opt_list, x_TO_list, mass_error_list

    for ar0 in AR_SWEEP:
        x0 = [v0, ar0]
        res = minimize(
            obj_mass_error1,
            x0,
            method="L-BFGS-B",
            bounds=bounds1,
            options={"ftol": 1e-1, "gtol": 1e-1, "maxfun": 25},
        )

        v_best, AR_best = res.x
        plane = Airplane(v_best, AR_best, W=weight)
        x_TO, masses = plane.runner()
        mass_error = abs(sum(masses) - mass_target)

        v_opt_list.append(v_best)
        AR_opt_list.append(AR_best)
        x_TO_list.append(x_TO)
        mass_error_list.append(mass_error)

    return v_opt_list, AR_opt_list, x_TO_list, mass_error_list


for mass_target in MASS_TARGETS:
    print(f"Optimizing for mass closure: {mass_target} [kg]")
    v_opt_list_ar, AR_opt_list_ar, x_TO_list_ar, mass_error_list_ar = [], [], [], []
    v_opt_list_ws, AR_opt_list_ws, x_TO_list_ws, mass_error_list_ws = [], [], [], []

    for v0 in V_SWEEP:
        weight = mass_target * g

        # aspect ratio
        v_opt_list_ar, AR_opt_list_ar, x_TO_list_ar, mass_error_list_ar = (
            optimizer_loop(
                v=v0,
                ar=True,
                weight=weight,
                v_opt_list=v_opt_list_ar,
                AR_opt_list=AR_opt_list_ar,
                x_TO_list=x_TO_list_ar,
                mass_error_list=mass_error_list_ar,
            )
        )

        # wing loading
        v_opt_list_ws, AR_opt_list_ws, x_TO_list_ws, mass_error_list_ws = (
            optimizer_loop(
                v=v0,
                ar=False,
                weight=weight,
                v_opt_list=v_opt_list_ws,
                AR_opt_list=AR_opt_list_ws,
                x_TO_list=x_TO_list_ws,
                mass_error_list=mass_error_list_ws,
            )
        )

    v_opt_list_ar = np.array(v_opt_list_ar)
    AR_opt_list_ar = np.array(AR_opt_list_ar)
    x_TO_list_ar = np.array(x_TO_list_ar)
    mass_error_list_ar = np.array(mass_error_list_ar)

    feasible_mask_ar = mass_error_list_ar < 50  # kg tolerance

    pareto_results_ar.append(
        {
            "mass_target": mass_target,
            "v_cruise": v_opt_list_ar[feasible_mask_ar],
            "AR": AR_opt_list_ar[feasible_mask_ar],
            "x_TO": x_TO_list_ar[feasible_mask_ar],
            "mass_error": mass_error_list_ar[feasible_mask_ar],
        }
    )

    v_opt_list_ws = np.array(v_opt_list_ws)
    AR_opt_list_ws = np.array(AR_opt_list_ws)
    x_TO_list_ws = np.array(x_TO_list_ws)
    mass_error_list_ws = np.array(mass_error_list_ws)

    feasible_mask_ws = mass_error_list_ws < 50  # kg tolerance

    pareto_results_ws.append(
        {
            "mass_target": mass_target,
            "v_cruise": v_opt_list_ws[feasible_mask_ws],
            "AR": AR_opt_list_ws[feasible_mask_ws],
            "x_TO": x_TO_list_ws[feasible_mask_ws],
            "mass_error": mass_error_list_ws[feasible_mask_ws],
        }
    )

# AR plot
plt.figure(figsize=(8, 6))
colors = plt.cm.viridis(np.linspace(0, 1, len(MASS_TARGETS)))

for i, res in enumerate(pareto_results_ar):
    plt.scatter(
        res["AR"],
        res["v_cruise"],
        s=50,
        color=colors[i],
        label=f"{(res['mass_target'])} kg",
    )

plt.xlabel("Aspect Ratio (AR)")
plt.ylabel("Cruise Velocity (m/s)")
plt.title("Pareto-like Front: v_cruise vs AR for each Mass Target")
plt.legend()
plt.grid(True)
plt.show()

# W/S variation plot
plt.figure(figsize=(8, 6))
for i, res in enumerate(pareto_results_ws):
    plt.scatter(
        res["v_cruise"],
        res["x_TO"],
        s=50,
        color=colors[i],
        label=f"{res['mass_target']} kg",
    )
    plt.plot(
        res["v_cruise"],
        res["x_TO"],
        color=colors[i],
    )

plt.xlabel("Cruise Velocity (m/s)")
plt.ylabel("Takeoff Distance x_TO (m)")
plt.title("Takeoff Distance vs Cruise Velocity (feasible designs)")
plt.legend()
plt.grid(True)
plt.show()
