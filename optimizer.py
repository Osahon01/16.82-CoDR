"""Optimization code for a small aircraft. Respects user-defined physical constraints."""

import matplotlib.pyplot as plt
import numpy as np
from pint import UnitRegistry
from pyomo.environ import *

# NOTE: You may need to install pyommo anad pint using pip

# Constants, see syllabus for constraints
ureg = UnitRegistry()  # keeps track of units
g = 9.81 * ureg("m/s^2")
N = 9  # 7-9 passengers
PAY = 100 * ureg("kg")  # per passenger
RANGE = 1500 * ureg("miles").to("m")
V_CRUISE = 125 * ureg("m/s")
X_TAKEOFF = 300 * ureg("ft").to("m")  # max takeoff distance; can be lower
# PAY_VOL = keep similar to those of similar-sized  aircrafts

try:
    # Objective function parameters
    cl_fn = ...

    model = ConcreteModel()
    model.cl = Var(bounds=(0.5, 15))  # inequality constraints
    model.n = Var(domain=Integers)  # type constraints
    model.cl_eq = Constraint(
        rule=cl_fn
    )  # will constrain optimized variables to r.s.p physical function constraint; explicit
    # MOCK: model._eq = Expression(expr=_fn)  # r.s.p physical function constraint; implicit

    # Objective function
    obj_fn = ...
except ValueError:
    print("\n*Need to implement variables*")

try:
    model.obj = Objective(expr=obj_fn, sense=minimize)
    solver = SolverFactory()  # read into options: bonmin, couenne, scip
    solver.solve(model)
    model.display()
except:
    print("\n*Implement objective function*\n")

if __name__ == "__main__":
    PLOT = False

    # visualization
    if PLOT:
        plt.figure()
        ...
        plt.show()
