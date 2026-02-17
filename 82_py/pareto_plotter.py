import numpy as np
import matplotlib.pyplot as plt


class TakeoffCruisePlotter:
    """
    Plots x_TO (roll takeoff distance, m) versus v_cruise (m/s) while sweeping v_cruise and AR.

    Assumptions:
      - airplane.py defines class Airplane(v_cruise, AR)
      - Airplane.runner() returns (x_TO, masses) where x_TO is scalar (meters)
      - If someone implemented runner as a *property* returning (x_TO, masses),
        this class also handles `obj.runner[0]` fallback.
    """

    def __init__(
        self,
        AirplaneClass,
        *,
        v_min: float,
        v_max: float,
        n_v: int,
        ar_values,
        ar_lines=None,
        # info box values (fixed design parameters)
        CLTO: float = None,
        CDTO: float = None,
        W: float = None,
        T_W: float = None,
    ):
        self.Airplane = AirplaneClass

        self.v_min = float(v_min)
        self.v_max = float(v_max)
        self.n_v = int(n_v)

        self.ar_values = np.array(list(ar_values), dtype=float)
        if self.ar_values.ndim != 1 or len(self.ar_values) < 2:
            raise ValueError(
                "ar_values must be a 1D list/array with at least 2 AR values."
            )

        self.v_values = np.linspace(self.v_min, self.v_max, self.n_v)

        # Which specific ARs to highlight as "contours vs v_cruise" (line plots)
        self.ar_lines = (
            np.array(list(ar_lines), dtype=float)
            if ar_lines is not None
            else np.array([], dtype=float)
        )

        # Annotation values
        self.CLTO = CLTO
        self.CDTO = CDTO
        self.W = W
        self.T_W = T_W

        # Computed results
        self.XTO = None  # shape (n_AR, n_V)

    def _get_x_to(self, airplane_obj) -> float:
        """Robustly extract x_TO from airplane_obj."""
        # Preferred: runner() method
        if callable(getattr(airplane_obj, "runner", None)):
            out = airplane_obj.runner()
            return float(out[0])

        # Fallback: runner property or attribute that is subscriptable (tuple/array)
        out = getattr(airplane_obj, "runner", None)
        if out is None:
            raise AttributeError(
                "Airplane object has no runner() method or runner attribute."
            )
        return float(out[0])

    def compute_grid(self, *, verbose: bool = True):
        """Compute x_TO over (AR, v_cruise) grid."""
        n_ar = len(self.ar_values)
        n_v = len(self.v_values)
        X = np.full((n_ar, n_v), np.nan, dtype=float)

        for i, AR in enumerate(self.ar_values):
            if verbose:
                print(f"Computing AR={AR:g} ({i + 1}/{n_ar})...")
            for j, v in enumerate(self.v_values):
                plane = self.Airplane(v_cruise=v, AR=AR)
                X[i, j] = self._get_x_to(plane)

        self.XTO = X
        return X

    def _annotation_text(self) -> str:
        lines = []
        if self.CLTO is not None:
            lines.append(f"CLTO = {self.CLTO:g}")
        if self.CDTO is not None:
            lines.append(f"CDTO = {self.CDTO:g}")
        if self.W is not None:
            lines.append(f"W = {self.W:,.0f} N")
        if self.T_W is not None:
            lines.append(f"T:W (TO) = {self.T_W:g}")
        return "\n".join(lines)

    def plot(
        self,
        *,
        levels=12,
        filled: bool = True,
        cmap: str = "viridis",
        show_ar_lines: bool = True,
        figsize=(10, 6),
        title: str = "Takeoff Distance vs Cruise Velocity and Aspect Ratio",
    ):
        """
        Creates a contour plot of x_TO over (v_cruise, AR),
        and optionally overlays x_TO vs v_cruise curves at specific ARs.
        """
        if self.XTO is None:
            self.compute_grid(verbose=True)

        V, AR = np.meshgrid(self.v_values, self.ar_values)  # shapes (n_AR, n_V)

        fig, ax = plt.subplots(figsize=figsize)

        # Contours of x_TO
        if filled:
            cf = ax.contourf(V, AR, self.XTO, levels=levels, cmap=cmap)
            cbar = fig.colorbar(cf, ax=ax)
            cbar.set_label("x_TO (m)")
            cs = ax.contour(
                V, AR, self.XTO, levels=levels, colors="k", linewidths=0.6, alpha=0.6
            )
            ax.clabel(cs, inline=True, fontsize=8, fmt="%.0f")
        else:
            cs = ax.contour(V, AR, self.XTO, levels=levels)
            ax.clabel(cs, inline=True, fontsize=8, fmt="%.0f")
            cbar = fig.colorbar(cs, ax=ax)
            cbar.set_label("x_TO (m)")

        # Overlay: x_TO vs v_cruise at specific ARs (your “contours at specific AR”)
        if show_ar_lines and self.ar_lines.size > 0:
            for ar_target in self.ar_lines:
                # Find nearest AR index on the grid
                idx = int(np.argmin(np.abs(self.ar_values - ar_target)))
                ar_used = self.ar_values[idx]
                ax.plot(
                    self.v_values, np.full_like(self.v_values, ar_used), lw=2, alpha=0.9
                )
                # Label the AR line on the right
                ax.text(
                    self.v_values[-1],
                    ar_used,
                    f"  AR={ar_used:g}",
                    va="center",
                    ha="left",
                    fontsize=9,
                    bbox=dict(
                        boxstyle="round,pad=0.2", fc="white", ec="none", alpha=0.7
                    ),
                )

        # Axes / title
        ax.set_title(title)
        ax.set_xlabel("v_cruise (m/s)")
        ax.set_ylabel("Aspect Ratio (AR)")
        ax.grid(True, alpha=0.25)

        # Info box (CLTO, CDTO, W, T:W)
        info = self._annotation_text()
        if info:
            ax.text(
                0.02,
                0.98,
                info,
                transform=ax.transAxes,
                va="top",
                ha="left",
                fontsize=10,
                bbox=dict(boxstyle="round,pad=0.35", fc="white", ec="0.5", alpha=0.9),
            )

        plt.tight_layout()
        return fig, ax

    def plot_x_to_vs_v_for_ARs(self, *, ar_list=None, figsize=(10, 5)):
        """
        Separate figure: x_TO vs v_cruise curves at selected ARs (line plot).
        Useful if you want the “specific aspect ratio contours” as classic curves.
        """
        if self.XTO is None:
            self.compute_grid(verbose=True)

        if ar_list is None:
            ar_list = self.ar_lines if self.ar_lines.size else self.ar_values

        ar_list = np.array(list(ar_list), dtype=float)

        fig, ax = plt.subplots(figsize=figsize)
        for ar_target in ar_list:
            idx = int(np.argmin(np.abs(self.ar_values - ar_target)))
            ar_used = self.ar_values[idx]
            ax.plot(self.v_values, self.XTO[idx, :], lw=2, label=f"AR={ar_used:g}")

        ax.set_title("x_TO vs v_cruise at Selected Aspect Ratios")
        ax.set_xlabel("v_cruise (m/s)")
        ax.set_ylabel("x_TO (m)")
        ax.grid(True, alpha=0.25)
        ax.legend()

        info = self._annotation_text()
        if info:
            ax.text(
                0.02,
                0.98,
                info,
                transform=ax.transAxes,
                va="top",
                ha="left",
                fontsize=10,
                bbox=dict(boxstyle="round,pad=0.35", fc="white", ec="0.5", alpha=0.9),
            )

        plt.tight_layout()
        return fig, ax


import math
from airplane import Airplane  # your class

plotter = TakeoffCruisePlotter(
    AirplaneClass=Airplane,
    v_min=80.0,
    v_max=140.0,
    n_v=11,
    ar_values=np.linspace(6, 18, 25),  # sweep AR for the contour grid
    ar_lines=[6, 8, 10, 12, 14],  # “specific ARs” to highlight
)

plotter.compute_grid(verbose=True)

# 2D contour map (v_cruise vs AR, contours are x_TO)
fig1, ax1 = plotter.plot(levels=15, filled=True)

# Optional: classic “contours at specific ARs” as x_TO-v curves
fig2, ax2 = plotter.plot_x_to_vs_v_for_ARs(ar_list=[8, 12, 16])
plt.show()
