import math


class ClimbModel:
    """
    Simple climb model with constant-density assumption evaluated at h_cruise.

    Inputs
    ------
    T_W : float
        Thrust-to-weight ratio (dimensionless).
    C_D : float
        Drag coefficient (dimensionless).
    S_W : float
        Wing loading expressed as S/W (m^2/N).
    gamma : float
        Climb angle (radians).
    P_generator : float
        Generator power (W).
    P_battery : float
        Battery power (W).
    eta_generator : float
        Generator efficiency (0–1).
    eta_battery : float
        Battery efficiency (0–1).
    epsilon_battery : float
        Battery energy density (J/kg).
    h_cruise : float
        Climb altitude in meters.
    """

    def __init__(
        self,
        T_W,
        C_D,
        S_W,
        gamma,
        P_generator,
        P_battery,
        eta_generator,
        eta_battery,
        epsilon_battery,
        h_cruise,
    ):
        self.T_W = T_W
        self.C_D = C_D
        self.S_W = S_W
        self.gamma = gamma
        self.P_generator = P_generator
        self.P_battery = P_battery
        self.eta_generator = eta_generator
        self.eta_battery = eta_battery
        self.epsilon_battery = epsilon_battery
        self.h_cruise = h_cruise

        self.g = 9.80665  # m/s^2

    def rho_isa(self):
        """
        ISA density at h_cruise (troposphere model).
        """
        h = self.h_cruise

        T0 = 288.15  # K
        p0 = 101325.0  # Pa
        L = 0.0065  # K/m
        R = 287.05287  # J/(kg*K)

        T = T0 - L * h
        if T <= 0:
            raise ValueError("Non-physical ISA temperature at this altitude.")

        p = p0 * (T / T0) ** (self.g / (R * L))
        rho = p / (R * T)
        return rho

    def combined_thrust_from_power(self, v):
        """
        Thrust from combined generator and battery power:
            T = (P_gen*eta_gen + P_bat*eta_bat) / v
        """
        if v <= 0:
            raise ValueError("Velocity must be positive.")
        P_eff = (
            self.P_generator * self.eta_generator + self.P_battery * self.eta_battery
        )
        return P_eff / v

    def solve_velocity(self):
        """
        Solves:
            T_W = 0.5*rho*v^2*(S/W)*C_D + sin(gamma)
        for v.
        """
        rho = self.rho_isa()
        sin_g = math.sin(self.gamma)

        numerator = self.T_W - sin_g
        denominator = 0.5 * rho * self.S_W * self.C_D

        if denominator <= 0:
            raise ValueError("Invalid aerodynamic parameters.")
        if numerator <= 0:
            raise ValueError("No real solution: T_W must exceed sin(gamma).")

        v = math.sqrt(numerator / denominator)
        return v

    def time_of_climb(self):
        """
        Computes:
            t_climb = h_cruise / (v * sin(gamma))
        """
        v = self.solve_velocity()
        sin_g = math.sin(self.gamma)

        if sin_g <= 0:
            raise ValueError("gamma must be positive for climb.")

        return self.h_cruise / (v * sin_g)

    def battery_mass_for_climb(self):
        """
        Computes battery mass required for climb:
            m_battery = (P_battery * t_climb) / epsilon_battery
        """
        if self.epsilon_battery <= 0:
            raise ValueError("epsilon_battery must be positive.")

        t_climb = self.time_of_climb()
        energy_required = self.P_battery * t_climb
        return energy_required / self.epsilon_battery

    # Main output getter
    def get_m_battery(self):
        """
        Returns battery mass required for climb (kg).
        """
        return self.battery_mass_for_climb()


# EXAMPLE USAGE

"""if __name__ == "__main__":
    gamma = math.radians(5.0)

    # 250 Wh/kg converted to J/kg
    epsilon_battery = 250.0 * 3600.0

    model = climb_model(
        T_W=0.30,
        C_D=0.03,
        S_W=1.2e-3,
        gamma=gamma,
        P_generator=80_000.0,
        P_battery=40_000.0,
        eta_generator=0.92,
        eta_battery=0.95,
        epsilon_battery=epsilon_battery,
        h_cruise=3048.0,  # 10,000 ft in meters
    )

    print(f"Climb velocity: {model.solve_velocity():.2f} m/s")
    print(f"Time to climb: {model.time_of_climb()/60:.2f} min")
    print(f"Battery mass required: {model.get_m_battery():.2f} kg")"""

# test comment