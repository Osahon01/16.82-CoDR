import numpy as np
from scipy.integrate import solve_ivp
from scipy.optimize import brentq
from CoDR_equations import g
from ambiance import Atmosphere


class TakeoffModel:
    """
    Units (consistent):
      W     : N
      W_S   : N/m^2
      S     : m^2 computed as W / W_S
      P_*   : W
      A_prop: m^2
      rho   : kg/m^3
      T     : N
      x_TO_target: m
    """

    def __init__(
        self,
        W_S,
        W,
        P_gen,
        x_TO_target,
        CLTO,
        CDTO,
        CMTO,
        AR,
        A_prop,
        eta_v=0.8,
        eta_add=0.9,
        t_max=100.0,
    ):
        self.W_S = float(W_S)
        self.W = float(W)
        self.P_gen = float(P_gen)
        self.x_TO_target = float(x_TO_target)

        self.CLTO = float(CLTO)
        self.CDTO = float(CDTO)
        self.CMTO = float(CMTO)
        self.AR = float(AR)
        self.A_prop = float(A_prop)

        self.g = float(g)
        self.rho_SL = float(Atmosphere(h=0).density[0])

        self.eta_v = float(eta_v)
        self.eta_add = float(eta_add)
        self.t_max = float(t_max)

        # Basic input sanity checks to prevent NaNs
        if self.W <= 0 or self.W_S <= 0:
            raise ValueError(f"W and W_S must be positive. Got W={self.W}, W_S={self.W_S}")
        if self.rho_SL <= 0:
            raise ValueError(f"Sea-level density must be positive. Got rho={self.rho_SL}")
        if self.A_prop <= 0:
            raise ValueError(f"A_prop must be positive (m^2). Got A_prop={self.A_prop}")
        if self.CLTO <= 0:
            raise ValueError(f"CLTO must be positive. Got CLTO={self.CLTO}")
        if self.CDTO < 0:
            raise ValueError(f"CDTO must be >= 0. Got CDTO={self.CDTO}")

        # Geometry from wing loading in N/m^2
        self.S = self.W / self.W_S
        self.b = np.sqrt(self.AR * self.S)
        self.c = self.S / self.b

        # Outputs (after solve)
        self.P_batt = None
        self.P_total = None
        self.T_takeoff = None
        self.TW_takeoff = None
        self.v_TO = None
        self.x_TO = None
        self.M_takeoff = None

    def _thrust_from_total_shaft_power(self, P_shaft_total: float) -> float:
        P = max(0.0, float(P_shaft_total))
        factor = np.sqrt(self.A_prop * self.rho_SL) * self.eta_v * self.eta_add
        T = (P * factor) ** (2.0 / 3.0)
        return float(T)

    def _simulate_takeoff_for_battery_power(self, P_batt: float):
        P_total = self.P_gen + float(P_batt)
        if P_total < 0:
            return np.inf, None, None, None, None

        T = self._thrust_from_total_shaft_power(P_total)
        TW = T / self.W

        if not np.isfinite(TW):
            return np.nan, None, None, None, None

        # Dynamics: y = [x, v]
        # Note: this is a pure aero drag model (no rolling resistance). If you want RR, add +mu*(W-L) term.
        def dyn(_t, y):
            x, v = y
            v = max(0.0, v)  # prevent negative velocity from numerical noise
            D_over_W = 0.5 * self.rho_SL * self.S / self.W * self.CDTO * v * v
            a = self.g * (TW - D_over_W)
            return [v, a]

        # Liftoff event: L - W = 0
        def liftoff_event(_t, y):
            v = max(0.0, y[1])
            L = 0.5 * self.rho_SL * v * v * self.S * self.CLTO
            return L - self.W

        liftoff_event.terminal = True
        liftoff_event.direction = 1

        y0 = [0.0, 0.0]

        sol = solve_ivp(
            dyn,
            (0.0, self.t_max),
            y0,
            events=liftoff_event,
            rtol=1e-6,
            atol=1e-8,
            method="RK45",
        )

        # If solver produced NaNs, treat as failure
        if not sol.success or np.any(~np.isfinite(sol.y)):
            return np.nan, None, None, TW, T

        if sol.t_events[0].size == 0:
            return np.inf, None, None, TW, T

        x_lo = float(sol.y_events[0][0][0])
        v_lo = float(sol.y_events[0][0][1])

        L_lo = 0.5 * self.rho_SL * v_lo * v_lo * self.S * self.CLTO
        M_lo = self.CMTO * L_lo * self.c

        return x_lo, v_lo, M_lo, TW, T

    def solve_for_battery_power(self, P_batt_max=5e6):
        x_target = float(self.x_TO_target)

        def f(Pb):
            x_lo, *_ = self._simulate_takeoff_for_battery_power(Pb)
            return x_lo - x_target

        f0 = f(0.0)

        # If NaN at zero power, it's an input/solver issue; raise with context
        if np.isnan(f0):
            raise RuntimeError(
                "Takeoff simulation returned NaN at P_batt=0. "
                "Check inputs/units (W in N, W_S in N/m^2, A_prop in m^2, CLTO/CDTO reasonable)."
            )

        # If generator alone cannot lift off within t_max, f0 will be +inf (finite check passes)
        if f0 <= 0.0:
            Pb_sol = 0.0
        else:
            Pb_hi = 1.0
            f_hi = f(Pb_hi)

            while (np.isnan(f_hi) or f_hi > 0.0) and Pb_hi < P_batt_max:
                Pb_hi *= 2.0
                f_hi = f(Pb_hi)

            if np.isnan(f_hi) or f_hi > 0.0:
                raise RuntimeError(
                    f"Could not achieve x_TO_target={x_target:.3f} m within P_batt_max={P_batt_max:.3g} W "
                    f"(or liftoff never occurs within t_max={self.t_max}s)."
                )

            Pb_sol = brentq(f, 0.0, Pb_hi, xtol=1e-6, rtol=1e-8, maxiter=200)

        x_lo, v_lo, M_lo, TW, T = self._simulate_takeoff_for_battery_power(Pb_sol)

        self.P_batt = float(Pb_sol)
        self.P_total = float(self.P_gen + self.P_batt)
        self.T_takeoff = float(T)
        self.TW_takeoff = float(TW)
        self.v_TO = None if v_lo is None else float(v_lo)
        self.x_TO = float(x_lo) if np.isfinite(x_lo) else float("inf")
        self.M_takeoff = None if M_lo is None else float(M_lo)

        return self.P_batt

    def get_battery_power(self):
        return self.P_batt
    
    def get_battery_mass_takeoff(
    self,
    specific_power=1500.0,   # W/kg (default: conservative aviation-grade)
    packaging_factor=1.25,   # accounts for structure, cooling, BMS, wiring
    ):
        """
        Returns battery mass (kg) sized by takeoff power requirement.

        Battery mass is computed as:
            m_batt = packaging_factor * P_batt / specific_power

        Assumes:
        - P_batt has already been solved via solve_for_battery_power()
        - specific_power is a rated continuous takeoff-capable value (W/kg)
        """

        if self.P_batt is None:
            raise RuntimeError(
                "Battery power not solved yet. "
                "Call solve_for_battery_power() first."
            )

        if specific_power <= 0.0:
            raise ValueError("specific_power must be positive (W/kg).")

        m_batt = self.P_batt / specific_power
        return packaging_factor * m_batt

    def get_TW_takeoff(self):
        return self.TW_takeoff

    def get_torsion_moment(self):
        return self.M_takeoff


# -----------------------------
# Known aircraft / mission data
# -----------------------------
W = 3600.0 * 9.81            # N, aircraft weight
W_S = 75 * 9.81            # N/m^2, wing loading
P_gen = 750000.0      # W, generator shaft power available at takeoff

x_TO_target = 10.0    # m, required takeoff ground roll

CLTO = 6.1             # takeoff lift coefficient
CDTO = 1.59            # takeoff drag coefficient
CMTO = 1.3            # nondimensional moment coefficient (model-specific)

AR = 8.0              # aspect ratio
A_prop = 3           # m^2, total prop disk area
'''
# -----------------------------
# Create takeoff model
# -----------------------------
to_model = TakeoffModel(
    W_S=W_S,
    W=W,
    P_gen=P_gen,
    x_TO_target=x_TO_target,
    CLTO=CLTO,
    CDTO=CDTO,
    CMTO=CMTO,
    AR=AR,
    A_prop=A_prop,
)

# -----------------------------
# Solve for required battery power
# -----------------------------
P_batt_req = to_model.solve_for_battery_power()

# -----------------------------
# Query results
# -----------------------------
print("=== Takeoff sizing results ===")
print(f"Target takeoff distance : {x_TO_target:.1f} m")
print(f"Achieved takeoff distance: {to_model.x_TO:.1f} m")
print(f"Generator power         : {P_gen/1e3:.1f} kW")
print(f"Required battery power  : {P_batt_req/1e3:.1f} kW")
print(f"Total shaft power       : {to_model.P_total/1e3:.1f} kW")
print(f"Takeoff thrust          : {to_model.T_takeoff:.0f} N")
print(f"Thrust-to-weight (T/W)  : {to_model.get_TW_takeoff():.3f}")
print(f'Wing area               : {to_model.S:.1f} m^2')

if to_model.v_TO is not None:
    print(f"Liftoff speed           : {to_model.v_TO:.1f} m/s")

if to_model.get_torsion_moment() is not None:
    print(f"Torsion moment @ TO     : {to_model.get_torsion_moment():.1f} N·m")

m_batt_TO = to_model.get_battery_mass_takeoff(
    specific_power=1500.0,   # W/kg
    packaging_factor=1.25
)

print(f"Battery mass (takeoff-sized): {m_batt_TO:.1f} kg")'''