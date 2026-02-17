"""NOTE: Do not run! For visualization, run mass_closure_post.py since data is already saved."""

import pickle
import numpy as np
from airplane import Airplane
import matplotlib.pyplot as plt
from scipy.optimize import minimize
from CoDR_equations import g


FILE_NAME = "82_py/airplane_optimization_results.pkl"
MASS_TARGETS = np.linspace(4000, 8000, 3)
V_SWEEP = np.linspace(50, 200, 10)

# for fixed run 1
AR_SWEEP = np.linspace(4, 15, 10)
W_S_FIXED = 130

# for fixed run 2
AR_FIXED = 9
W_S_SWEEP = np.linspace(50, 90, 10)

pareto_results_ar = []
pareto_results_ws = []

bounds1 = [(50, 130), (4, 15)]  # vcruise and AR ICS
bounds2 = [(50, 130), (50, 90)]  # vcruise and W_S ICS


def run_closure():
    """Runs optimizer for mass closure model. Residual is defined as |m_target - m_value|."""

    def make_obj_mass_error1(weight, mass_target):
        def obj(x):
            v, ar = x

            plane = Airplane(
                v,
                AR=ar,
                W=weight,
                W_S=W_S_FIXED,
            )
            _, masses = plane.runner()
            m = np.sum(masses)
            return np.abs(m - mass_target)  # normalized for stability

        return obj

    def make_obj_mass_error2(weight, mass_target):
        def obj(x):
            v, w_s = x

            plane = Airplane(
                v,
                AR=AR_FIXED,
                W=weight,
                W_S=w_s,
            )

            _, masses = plane.runner()
            m = np.sum(masses)
            return np.abs(m - mass_target)

        return obj

    def optimizer_loop(
        v=False,
        ar=False,
        weight=0.0,
        v_opt_list=None,
        AR_opt_list=None,
        x_TO_list=None,
        mass_error_list=None,
    ):
        if v_opt_list is None:
            v_opt_list = []
            AR_opt_list = []
            x_TO_list = []
            mass_error_list = []

        # w/s optimization
        if not ar:
            obj = make_obj_mass_error2(weight, mass_target)
            for ws0 in W_S_SWEEP:
                x0 = [v0, ws0]
                res = minimize(
                    obj,
                    x0,
                    method="L-BFGS-B",
                    bounds=bounds2,
                    options={
                        "ftol": 1e-3,
                        "gtol": 1e-3,
                        "maxfun": 500,
                    },
                )

                if not res.success:
                    continue

                v_best, w_s_best = res.x

                plane = Airplane(
                    v_best,
                    AR=AR_FIXED,
                    W=weight,
                    W_S=w_s_best,
                )

                x_TO, masses = plane.runner()

                mass_error = abs(np.sum(masses) - mass_target)

                v_opt_list.append(v_best)
                AR_opt_list.append(w_s_best)
                x_TO_list.append(x_TO)
                mass_error_list.append(mass_error)

            return v_opt_list, AR_opt_list, x_TO_list, mass_error_list

        # AR optimization
        obj = make_obj_mass_error1(weight, mass_target)

        for ar0 in AR_SWEEP:
            x0 = [v0, ar0]

            res = minimize(
                obj,
                x0,
                method="L-BFGS-B",
                bounds=bounds1,
                options={
                    "ftol": 1e-3,
                    "gtol": 1e-3,
                    "maxfun": 500,
                },
            )

            if not res.success:
                continue

            v_best, AR_best = res.x

            plane = Airplane(
                v_best,
                AR=AR_best,
                W=weight,
            )

            x_TO, masses = plane.runner()

            mass_error = abs(np.sum(masses) - mass_target)

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

            # wing loading opt
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

            # ar optimization
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

        v_opt_list_ws = np.array(v_opt_list_ws)
        AR_opt_list_ws = np.array(AR_opt_list_ws)
        x_TO_list_ws = np.array(x_TO_list_ws)
        mass_error_list_ws = np.array(mass_error_list_ws)

        feasible_mask_ws = mass_error_list_ws < 50

        pareto_results_ws.append(
            {
                "mass_target": mass_target,
                "v_cruise": v_opt_list_ws[feasible_mask_ws],
                "AR": AR_opt_list_ws[feasible_mask_ws],
                "x_TO": x_TO_list_ws[feasible_mask_ws],
                "mass_error": mass_error_list_ws[feasible_mask_ws],
            }
        )

        v_opt_list_ar = np.array(v_opt_list_ar)
        AR_opt_list_ar = np.array(AR_opt_list_ar)
        x_TO_list_ar = np.array(x_TO_list_ar)
        mass_error_list_ar = np.array(mass_error_list_ar)

        feasible_mask_ar = mass_error_list_ar < 50

        pareto_results_ar.append(
            {
                "mass_target": mass_target,
                "v_cruise": v_opt_list_ar[feasible_mask_ar],
                "AR": AR_opt_list_ar[feasible_mask_ar],
                "x_TO": x_TO_list_ar[feasible_mask_ar],
                "mass_error": mass_error_list_ar[feasible_mask_ar],
            }
        )

    # save output vectors
    output_data = {
        "pareto_results_ar": pareto_results_ar,
        "pareto_results_ws": pareto_results_ws,
    }
    with open(FILE_NAME, "wb") as f:
        pickle.dump(output_data, f)

    print(f"\nOptimization results successfully saved to {FILE_NAME}")


if __name__ == "__main__":
    run_closure()
