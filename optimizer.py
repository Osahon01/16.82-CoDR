import matplotlib.pyplot as plt
import numpy as np
from pint import UnitRegistry

ureg = UnitRegistry()  # keeps track of units

# define module level variables here
PLOT = False

# target constraints here

# objective function

# initialization
f_init = np.random.rand()
print(f"{30 * '#'} \nObj. fn initialized to {np.round(f_init, 3)} \n{30 * '#'}")

# optimizer(s)


if __name__ == "__main__":
    print("Need to implement")

    # visualization
    if PLOT:
        plt.figure()
        ...
        plt.show()
