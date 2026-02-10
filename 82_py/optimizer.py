"""Optimization code for a small aircraft. Respects user-defined physical constraints."""

import numpy as np
import matplotlib.pyplot as plt
from pint import UnitRegistry
from pyomo.environ import *
from CoDR_equations import *

try:
    # Objective function parameters
    cl_fn = ...

    model = ConcreteModel()
    model.cl = Var(bounds=(0.5, 15.0))  # inequality constraints
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
