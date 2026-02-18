"""NOTE: Do not run! For visualization, run mass_closure_post.py since data is already saved."""

import pickle
import numpy as np
from airplane import Airplane
import matplotlib.pyplot as plt
from scipy.optimize import minimize
from CoDR_equations import g


FILE_NAME = "82_py/airplane_optimization_results.pkl"
MASS_TARGETS = np.linspace(4000, 6000, 3)
V_SWEEP = np.linspace(50, 130, 5)

# for fixed run 1
AR_SWEEP = np.linspace(4, 15, 5)
W_S_FIXED = 130
XT0_FIXED = 300.0 / 3.28

# for fixed run 2
AR_FIXED = 9
XT0_SWEEP = np.linspace(50, 90, 5)

pareto_results_ar = []
pareto_results_xt0 = []

bounds1 = [(50, 130), (4, 15)]  # vcruise and AR ICS
bounds2 = [(50, 130), (50, 90)]  # vcruise and xt0 ICS


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
            masses = plane.runner()
            m = np.sum(masses)
            return np.abs(m - mass_target)  # normalized for stability

        return obj

    def make_obj_mass_error2(weight, mass_target):
        def obj(x):
            v, xTO = x

            plane = Airplane(
                v,
                AR=AR_FIXED,
                W=weight,
                W_S=W_S_FIXED,
                xTO=xTO,
            )

            masses = plane.runner()
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
            print("\n  Starting xt0 Sweep")
            for xt0 in XT0_SWEEP:
                print(f"        iteration for xt0: {xt0}\n")
                x0 = [v0, xt0]
                res = minimize(
                    obj,
                    x0,
                    method="Nelder-Mead",
                    # method="L-BFGS-B",
                    bounds=bounds2,
                    options={
                        "ftol": 1e-7,
                        "gtol": 1e-7,
                        "maxfun": 500,
                    },
                )

                if not res.success:
                    continue

                v_best, xt0_best = res.x

                plane = Airplane(
                    v_best,
                    AR=AR_FIXED,
                    W=weight,
                    xTO=xt0_best,
                )

                masses = plane.runner()

                mass_error = abs(np.sum(masses) - mass_target)

                v_opt_list.append(v_best)
                AR_opt_list.append(xt0_best)
                x_TO_list.append(xt0)
                mass_error_list.append(mass_error)

            return v_opt_list, AR_opt_list, x_TO_list, mass_error_list

        # AR optimization
        obj = make_obj_mass_error1(weight, mass_target)

        print("\n  Starting AR Sweep")
        for ar0 in AR_SWEEP:
            x0 = [v0, ar0]
            print(f"        iteration for ar0: {ar0}\n")

            res = minimize(
                obj,
                x0,
                method="Nelder-Mead",
                # method="L-BFGS-B",
                bounds=bounds1,
                options={
                    "ftol": 1e-7,
                    "gtol": 1e-7,
                    "maxfun": 500,
                },
            )

            if not res.success:
                continue

            v_best, AR_best = res.x

            plane = Airplane(v_best, AR=AR_best, W=weight, xTO=XT0_FIXED)

            masses = plane.runner()

            mass_error = abs(np.sum(masses) - mass_target)

            v_opt_list.append(v_best)
            AR_opt_list.append(AR_best)
            x_TO_list.append(XT0_FIXED)
            mass_error_list.append(mass_error)

        return v_opt_list, AR_opt_list, x_TO_list, mass_error_list

    for idx_mass, mass_target in enumerate(MASS_TARGETS):
        print(
            f"Optimizing {idx_mass / len(MASS_TARGETS)} for mass closure: {mass_target} [kg]"
        )

        v_opt_list_ar, AR_opt_list_ar, x_TO_list_ar, mass_error_list_ar = [], [], [], []
        v_opt_list_xt0, AR_opt_list_xt0, x_TO_list_xt0, mass_error_list_xt0 = (
            [],
            [],
            [],
            [],
        )

        for idx_v0, v0 in enumerate(V_SWEEP):
            print(f"\n    iteration {idx_v0} - velocity: {v0} [m/s]\n")
            weight = mass_target * g

            # wing loading opt
            v_opt_list_xt0, AR_opt_list_xt0, x_TO_list_xt0, mass_error_list_xt0 = (
                optimizer_loop(
                    v=v0,
                    ar=False,
                    weight=weight,
                    v_opt_list=v_opt_list_xt0,
                    AR_opt_list=AR_opt_list_xt0,
                    x_TO_list=x_TO_list_xt0,
                    mass_error_list=mass_error_list_xt0,
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

        v_opt_list_xt0 = np.array(v_opt_list_xt0)
        AR_opt_list_xt0 = np.array(AR_opt_list_xt0)
        x_TO_list_xt0 = np.array(x_TO_list_xt0)
        mass_error_list_xt0 = np.array(mass_error_list_xt0)

        feasible_mask_xt0 = mass_error_list_xt0 < 50

        pareto_results_xt0.append(
            {
                "mass_target": mass_target,
                "v_cruise": v_opt_list_xt0[feasible_mask_xt0],
                "AR": AR_opt_list_xt0[feasible_mask_xt0],
                "x_TO": x_TO_list_xt0[feasible_mask_xt0],
                "mass_error": mass_error_list_xt0[feasible_mask_xt0],
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
                "x_TO": XT0_FIXED,
                "mass_error": mass_error_list_ar[feasible_mask_ar],
            }
        )

    # save output vectors
    output_data = {
        "pareto_results_ar": pareto_results_ar,
        "pareto_results_xt0": pareto_results_xt0,
    }
    with open(FILE_NAME, "wb") as f:
        pickle.dump(output_data, f)

    print(f"\nOptimization results successfully saved to {FILE_NAME}")


if __name__ == "__main__":
    run_closure()
