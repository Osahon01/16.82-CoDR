from math import sqrt, pi
from math import radians


class StructuralWingModel:
    """
    First-pass structural mass model for a (single-cell) wingbox:
      - Spar caps sized by wing root bending from lift (elliptical spanwise load)
      - Closed-section skin sized by wing torsion (thin-walled torsion)

    Inputs (SI units):
      L_max      : N      total lift at sizing condition
      M_max      : N*m    total wing torsion torque
      S          : m^2    wing planform area
      AR         : -      aspect ratio
      FOS        : -      factor of safety
      rho_spar   : kg/m^3 spar composite density
      rho_skin   : kg/m^3 skin composite density
    """

    def __init__(
        self,
        L_max,
        M_max,
        S,
        AR,
        FOS,
        rho_spar,
        rho_skin,
        # (From Google) "Medium quality" carbon composite design allowables
        sigma_allow_design=450e6,  # Pa
        tau_allow_design=80e6,  # Pa
        # Wingbox geometry assumptions
        box_width_frac=0.45,
        box_height_frac=0.12,
        # Skin minimum gauge
        min_skin_thickness=0.0012,
    ):
        # Primary inputs
        self.L_max = L_max
        self.M_max = M_max
        self.S = S
        self.AR = AR
        self.FOS = FOS
        self.rho_spar = rho_spar
        self.rho_skin = rho_skin

        # Material allowables
        self.sigma_allow_design = sigma_allow_design
        self.tau_allow_design = tau_allow_design

        # Geometry assumptions
        self.box_width_frac = box_width_frac
        self.box_height_frac = box_height_frac

        # Manufacturing / buckling floor
        self.min_skin_thickness = min_skin_thickness

        self._validate()

    def _validate(self):
        for name, v in [
            ("L_max", self.L_max),
            ("M_max", self.M_max),
            ("S", self.S),
            ("AR", self.AR),
            ("FOS", self.FOS),
            ("rho_spar", self.rho_spar),
            ("rho_skin", self.rho_skin),
            ("sigma_allow_design", self.sigma_allow_design),
            ("tau_allow_design", self.tau_allow_design),
        ]:
            if v <= 0:
                raise ValueError(f"{name} must be > 0, got {v}")

    @property
    def span(self) -> float:
        """Wing span b from AR and S: AR = b^2 / S."""
        self._validate()
        return sqrt(self.AR * self.S)

    @property
    def mean_chord(self) -> float:
        """Mean chord c̄ ~ S / b (rectangular-equivalent)."""
        b = self.span
        return self.S / b

    def root_bending_moment_elliptical(self) -> float:
        """
        Root bending moment from total lift assuming *elliptical* lift per unit span.
        For a symmetric wing, treat each half-wing as a cantilever beam with distributed load w(y).

        If total lift is L over full span b, each half-wing carries L/2.
        For an elliptical distribution on half-span (0..b/2), the root moment evaluates to:

          M_root = (L * b) / (3π)

        This is lower than the uniform-load result (L*b/8) and is commonly used for first sizing.
        """
        b = self.span
        return (self.L_max * b) / (3.0 * pi)

    def spar_mass(self) -> float:
        """
        Spar caps sized by bending using a simple two-cap model.

        Assume bending carried by two spar caps separated by wingbox height h.
        Using I ≈ A_cap * h^2 / 2 (two caps) leads to cap stress:

          σ ≈ M_root / (A_cap * h)

        => A_cap >= M_root / (σ_allow * h)

        Total spar area ~ 2 * A_cap (no explicit web mass here; add later if you want).
        Mass assumes constant section along the full span (very rough).
        """
        self._validate()

        b = self.span
        c = self.mean_chord
        h = self.box_height_frac * c
        if h <= 0:
            raise ValueError(
                "Computed wingbox height is non-positive; check geometry fractions."
            )

        M_root = self.root_bending_moment_elliptical()

        sigma_allow = self.sigma_allow_design / self.FOS
        A_cap = M_root / (sigma_allow * h)  # m^2 per cap

        A_total = 2.0 * A_cap
        volume = A_total * b
        return self.rho_spar * volume

    def skin_mass(self) -> float:
        """
        Skin sized by torsion using thin-walled closed section theory (single-cell wingbox).
        Using shear flow approximation from 16.20
        For a thin-walled closed cell:
          shear flow q = T / (2 A_m)
          shear stress τ = q / t  =>  t >= T / (2 A_m tau_allow)

        Use rectangular median area A_m ≈ w*h, perimeter p = 2(w+h).
        Skin volume ≈ (p * t) * b (constant thickness along span).
        """
        self._validate()

        b = self.span
        c = self.mean_chord
        w = self.box_width_frac * c
        h = self.box_height_frac * c
        if w <= 0 or h <= 0:
            raise ValueError(
                "Computed wingbox dimensions are non-positive; check geometry fractions."
            )

        A_m = w * h
        p = 2.0 * (w + h)

        tau_allow = self.tau_allow_design / self.FOS
        t_req = self.M_max / (2.0 * A_m * tau_allow)

        t = max(t_req, self.min_skin_thickness)
        volume = (p * t) * b
        return self.rho_skin * volume

    def mass_breakdown(self) -> tuple[float, float]:
        """Return (m_spar, m_skin) in kg."""
        return self.spar_mass(), self.skin_mass()

    # Example sizing point (SI units)


# Suppose:
# - L_max is total lift at the sizing condition (N)
# - M_max is total torsional torque to be reacted by the wingbox (N*m)
# - S is wing area (m^2)
# - AR is aspect ratio (-)
# - FOS is factor of safety (-)
# - rho_* are material densities (kg/m^3)


# Paste/import the structural_model class definition above before running this.

model = StructuralWingModel(
    L_max=120_000.0,  # N
    M_max=30_000.0,  # N*m (torsion torque, NOT bending)
    S=28.0,  # m^2
    AR=9.5,  # -
    FOS=1.5,  # -
    # "Medium quality" carbon composite-ish densities (typical range ~1500-1700)
    rho_spar=1600.0,  # kg/m^3
    rho_skin=1550.0,  # kg/m^3
    # Optional: override allowables if you have better laminate allowables
    sigma_allow_design=450e6,  # Pa
    tau_allow_design=80e6,  # Pa
)

m_spar, m_skin = model.mass_breakdown()

print(f"Span b              : {model.span:.3f} m")
print(f"Mean chord c_bar    : {model.mean_chord:.3f} m")
print(f"Root bending moment : {model.root_bending_moment_elliptical():.1f} N*m")
print(f"Estimated spar mass : {m_spar:.2f} kg")
print(f"Estimated skin mass : {m_skin:.2f} kg")
print(f"Total (spar+skin)   : {(m_spar + m_skin):.2f} kg")
