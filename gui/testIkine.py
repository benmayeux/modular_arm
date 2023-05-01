import math

import numpy as np
import roboticstoolbox as rtb

# some measurements that won't change for our physical version
lengthOfRLinks = 0.211201  # in m
baseHOffset = 0.049276
baseVOffset = 0.2014855

robot = rtb.DHRobot([
            rtb.RevoluteDH(d=baseVOffset, a=baseHOffset, alpha=-math.pi / 2),
            rtb.RevoluteDH(a=lengthOfRLinks),
            rtb.RevoluteDH(a=lengthOfRLinks)
        # rtb.RevoluteMDH(d=baseVOffset, a=baseHOffset),
        # rtb.RevoluteMDH(alpha=-math.pi/2),
        # rtb.RevoluteMDH(a=lengthOfRLinks),
        # rtb.RevoluteMDH(a=lengthOfRLinks)
        ], name="RRR")

T = robot.fkine([-math.pi/4, -math.pi/4, math.pi/3])
print(T)
# inLimits = False
# while not inLimits:
#     inLimits = True
#     sol = robot.ikine_LM(T, L=np.random.random(), mask=[1,1,1,0,0,0], q0=robot.qrandom)#transpose=False, ilimit=1000, search=True)
#     for i in sol[0]:
#         if abs(i) >= math.pi/2:
#             inLimits = False
#
#     print(robot.fkine(sol[0]))
#     print(sol)

# sol = robot.ik_nr(T)
try:
    sol = robot.ikine_LM(T, search=True)
except:
    sol = robot.ikine_LM(T, search=True, q0=[math.pi,0,0])
print(robot.fkine(sol[0]))
print(sol)



# puma = rtb.models.DH.Panda()
# T = puma.fkine([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7])  # forward kinematics
# print(T)
# sol = puma.ik_nr(T)                          # inverse kinematics
# print(sol)