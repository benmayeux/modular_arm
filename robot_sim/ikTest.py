import numpy as np
import roboticstoolbox as rtb
import time
from ansitable import ANSITable

from numpy import ndarray
import numpy as np
from spatialmath import SE3
from typing import Union, overload, List, Set

lengthOfRLinks = 0.211201 # in m
baseHOffset = 0.049276
baseVOffset = 0.2014855

robot = rtb.DHRobot([
            rtb.RevoluteDH(d=baseVOffset, a=baseHOffset, alpha=-np.pi / 2),
            rtb.RevoluteDH(a=lengthOfRLinks),
            rtb.RevoluteDH(a=lengthOfRLinks)
        ])

homePos = robot.fkine([0,0,0])
print("HomePos: \n", homePos)
desPos = robot.fkine([-1.57,.785,-.785])
print("desPos: \n", desPos)
calcThetas = robot.ikine(desPos)[0]
print("Calc Thetas: \n",calcThetas)

calcPos = robot.fkine(calcThetas)
print("calcPos: \n", calcPos)

def test():

    # Our robot and ETS
    robot = rtb.models.Panda()
    ets = robot.ets()

    ### Experiment parameters
    # Number of problems to solve
    problems = 10000

    # Cartesion DoF priority matrix
    we = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])

    # random valid q values which will define Tep
    q_rand = ets.random_q(problems)

    # Our desired end-effector poses
    Tep = np.zeros((problems, 4, 4))

    for i in range(problems):
        Tep[i] = ets.eval(q_rand[i])

    # Maximum iterations allowed in a search
    ilimit = 30

    # Maximum searches allowed per problem
    slimit = 100

    # Solution tolerance
    tol = 1e-6


    solvers = [
        lambda Tep: ets.ik_nr(
            Tep,
            q0=None,
            ilimit=ilimit,
            slimit=slimit,
            tol=tol,
            reject_jl=False,
            we=we,
            use_pinv=True,
            pinv_damping=0.0,
        ),
        lambda Tep: ets.ik_gn(
            Tep,
            q0=None,
            ilimit=ilimit,
            slimit=slimit,
            tol=tol,
            reject_jl=False,
            we=we,
            use_pinv=False,
            pinv_damping=0.2,
        ),
        lambda Tep: ets.ik_lm_chan(
            Tep,
            q0=None,
            ilimit=ilimit,
            slimit=slimit,
            tol=tol,
            reject_jl=True,
            we=we,
            λ=0.1,
        ),
        lambda Tep: ets.ik_lm_wampler(
            Tep,
            q0=None,
            ilimit=ilimit,
            slimit=slimit,
            tol=tol,
            reject_jl=True,
            we=we,
            λ=1e-4,
        ),
        lambda Tep: ets.ik_lm_sugihara(
            Tep,
            q0=None,
            ilimit=ilimit,
            slimit=slimit,
            tol=tol,
            reject_jl=True,
            we=we,
            λ=0.1,
        ),
    ]

    times = []

    solver_names = [
        "Newton Raphson",
        "Gauss Newton",
        "LM Chan",
        "LM Wampler",
        "LM Sugihara",
    ]

    for solver in solvers:
        print("Next Solver")

        start = time.time()

        for i in range(problems):
            solver(Tep[i])

        total_time = time.time() - start
        times.append(total_time)


    print(f"\nNumerical Inverse Kinematics Methods Compared over {problems} problems\n")

    table = ANSITable(
        "Method",
        "Time",
        border="thin",
    )

    for name, t in zip(solver_names, times):
        table.row(
            name,
            (t / problems) * 1e6,
        )

    table.print()