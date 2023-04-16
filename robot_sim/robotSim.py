import pybullet as p
import time
import pybullet_data
import matplotlib.pyplot as plt
import numpy as np
import math as m

def main():
    physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
    p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
    print(pybullet_data.getDataPath())
    p.setGravity(0,0,-9.81)
    p.resetDebugVisualizerCamera( cameraDistance=2, cameraYaw=140, cameraPitch=-40, cameraTargetPosition=[0,0,0])
    p.setTimeStep(1./240.)
    planeId = p.loadURDF("plane.urdf")
    startPos = [0,0,.5]
    startOrientation = p.getQuaternionFromEuler([0,0,0])
    robotID = p.loadURDF(r"C:\Users\salva\Documents\WPI\Modular Arm\threeSerial.URDF",startPos, startOrientation,useFixedBase=1)
    mode = p.setJointMotorControlArray(robotID, [0,1,2],controlMode=p.VELOCITY_CONTROL, forces=[0,0,0])
    #set the center of mass frame (loadURDF sets base link frame) startPos/Ornp.resetBasePositionAndOrientation(boxId, startPos, startOrientation)
    currT = 0
    #Gains for old EoM
    #Kd = [.25,5,2.5]
    #Kp = [10,15,25]
    #Gains for new EoM - Need work
    Kd = [.15,15,7.5]
    Kp = [10,75,75]

    targetLog = []
    thetaLog = []
    while currT<5.1:
        desiredPos = [1.57,-.785,-.785]
        #For testing
        targetJoint = 0
        #Get current pos and vel for all joints
        robotPos,robotVel = getJointStates(robotID)
        U,targets = calcFeedbackLinearization(Kp,Kd,desiredPos,robotPos,robotVel,0,currT)
        
        if currT<5: mode = p.setJointMotorControlArray(robotID,[0,1,2], controlMode=p.TORQUE_CONTROL, forces=U)
        else: mode = p.setJointMotorControlArray(robotID, [0,1,2], controlMode=p.VELOCITY_CONTROL, forces=[0,0,0])

        desiredThetas = traj_evaluate(0,5,currT,[0,0,0],desiredPos)
        #Logging
        targetLog.append(desiredThetas[0,targetJoint])
        thetaLog.append(robotPos[targetJoint])

        if round(currT%.1,2)==0: 
            print("Time: ",round(currT,2))
            print("JointPos: ",robotPos[0])
            print("Joint Target: ",targets[0,0])
            print("Control Inputs: ",U)

        #time.sleep(1./240.)
        currT += 1./240.
        p.stepSimulation()
    cubePos, cubeOrn = p.getBasePositionAndOrientation(robotID)
    p.disconnect()
    return targetLog,thetaLog
    

def getJointStates(robotID):
    #Get all state info 
    joint1State = p.getJointState(robotID,0)
    joint2State = p.getJointState(robotID,1)
    joint3State = p.getJointState(robotID,2)
    
    joint1Pos = joint1State[0]
    joint1Vel = joint1State[1]
    joint2Pos = joint2State[0]
    joint2Vel = joint2State[1]
    joint3Pos = joint3State[0]
    joint3Vel = joint3State[1]

    robotPos = [joint1Pos,joint2Pos,joint3Pos]
    robotVel = [joint1Vel,joint2Vel,joint3Vel] 

    return robotPos,robotVel

# Returns point on quintic trajectory in 3D space
def traj_evaluate(t0,tf,currT,thetaStart,thetaFinal):
        Amat = np.array([[1, t0, t0**2, t0**3, t0**4, t0**5],
        [0, 1, 2*t0, 3*t0**2, 4*t0**3, 5*t0**4],
        [0, 0, 2, 6*t0, 12*t0**2, 20*t0**3],
        [1, tf, tf**2, tf**3, tf**4, tf**5],
        [0, 1, 2*tf, 3*tf**2, 4*tf**3, 5*tf**4],
        [0, 0, 2, 6*tf, 12*tf**2, 20*tf**3]])

        t1Coeffs = np.matmul(np.linalg.inv(Amat), np.array([[thetaStart[0]],[0],[0],[thetaFinal[0]],[0],[0]]))
        t2Coeffs = np.matmul(np.linalg.inv(Amat), np.array([[thetaStart[1]],[0],[0],[thetaFinal[1]],[0],[0]]))
        t3Coeffs = np.matmul(np.linalg.inv(Amat), np.array([[thetaStart[2]],[0],[0],[thetaFinal[2]],[0],[0]]))
        coEffs = np.concatenate((t1Coeffs,t2Coeffs,t3Coeffs),axis=1)

        A = np.array([[1, currT, currT**2, currT**3, currT**4, currT**5],
            [0, 1, 2*currT, 3*currT**2, 4*currT**3, 5*currT**4],
            [0, 0, 2, 6*currT, 12*currT**2, 20*currT**3]])

        traj = np.zeros([3,3])
        traj[:,0] = np.matmul(A,coEffs[:,0])
        traj[:,1] = np.matmul(A,coEffs[:,1])
        traj[:,2] = np.matmul(A,coEffs[:,2])

        return traj
def calcFeedbackLinearization(Kp,Kd,desPos,curPos,curVel,startT,currT):
    linkLength = .2
    linkCenter = .1
    linkMass = .5
    g = 9.81
    linkI = .1
    U = [0,0,0]


    desiredThetas = traj_evaluate(0,5,currT,[0,0,0],desPos)

    posError = [desiredThetas[0,0] - curPos[0], desiredThetas[0,1] - curPos[1], desiredThetas[0,2] - curPos[2]]
    #posError = [x*-1 for x in posError]
    velError = [desiredThetas[1,0] - curVel[0],desiredThetas[1,1] - curVel[1],desiredThetas[1,2] - curVel[2]]
    #velError = [x*-1 for x in velError]
    #Virtual Control Input
    v = [0,0,0]
    v[0] = posError[0]*Kp[0] + velError[0]*Kd[0] + desiredThetas[0,2]
    v[1] = posError[1]*Kp[1] + velError[1]*Kd[1] + desiredThetas[1,2]
    v[2] = posError[2]*Kp[2] + velError[2]*Kd[2] + desiredThetas[2,2]
    
    #Old Feedback linearization
    #U[0] = v[0]*(linkI + linkI + linkI + (linkLength*linkLength*linkMass)/2 + (linkCenter*linkCenter*linkMass)/2 + (linkCenter*linkCenter*linkMass)/2 - (linkLength*linkLength*linkMass*m.cos(2*curPos[1]))/2 - (linkCenter*linkCenter*linkMass*m.cos(2*curPos[1]))/2 + (linkCenter*linkCenter*linkMass*m.cos(2*curPos[1] + 2*curPos[2]))/2 + linkLength*linkCenter*linkMass*m.sin(curPos[2]) - linkLength*linkCenter*linkMass*m.sin(2*curPos[1] + curPos[2])) + (curVel[0]*curVel[1]*(2*linkMass*m.sin(2*curPos[1])*linkLength*linkLength - 4*linkMass*m.cos(2*curPos[1] + curPos[2])*linkLength*linkCenter + 2*linkMass*m.sin(2*curPos[1])*linkCenter*linkCenter - 2*linkMass*m.sin(2*curPos[1] + 2*curPos[2])*linkCenter*linkCenter))/2 - linkCenter*linkMass*curVel[0]*curVel[2]*(linkCenter*m.sin(2*curPos[1] + 2*curPos[2]) - linkLength*m.cos(curPos[2]) + linkLength*m.cos(2*curPos[1] + curPos[2]))
    #U[1] = v[2]*(linkMass*linkCenter*linkCenter + linkLength*linkMass*np.sin(curPos[2])*linkCenter + linkI) + v[1]*(linkMass*linkLength*linkLength + 2*linkMass*np.sin(curPos[2])*linkLength*linkCenter + linkMass*linkCenter*linkCenter + linkMass*linkCenter*linkCenter + linkI + linkI) + linkCenter*g*linkMass*np.cos(curPos[1] + curPos[2]) - linkLength*g*linkMass*np.sin(curPos[1]) - linkCenter*g*linkMass*np.sin(curPos[1]) - (linkLength*linkLength*linkMass*curVel[0]*curVel[0]*np.sin(2*curPos[1]))/2 - (linkCenter*linkCenter*linkMass*curVel[0]*curVel[0]*np.sin(2*curPos[1]))/2 + (linkCenter*linkCenter*linkMass*curVel[0]*curVel[0]*np.sin(2*curPos[1] + 2*curPos[2]))/2 + linkLength*linkCenter*linkMass*curVel[0]*curVel[0]*np.cos(2*curPos[1] + curPos[2]) + linkLength*linkCenter*linkMass*curVel[2]*np.cos(curPos[2])*(2*curVel[1] + curVel[2]);
    #U[2] = linkI*v[1] + linkI*v[2] + linkCenter*linkCenter*linkMass*v[1] + linkCenter*linkCenter*linkMass*v[2] + linkCenter*g*linkMass*np.cos(curPos[1] + curPos[2]) + (linkCenter*linkCenter*linkMass*curVel[0]*curVel[0]*np.sin(2*curPos[1] + 2*curPos[2]))/2 - (linkLength*linkCenter*linkMass*curVel[0]*curVel[0]*np.cos(curPos[2]))/2 - linkLength*linkCenter*linkMass*curVel[1]*curVel[1]*np.cos(curPos[2]) + (linkLength*linkCenter*linkMass*curVel[0]*curVel[0]*np.cos(2*curPos[1] + curPos[2]))/2 + linkLength*linkCenter*linkMass*v[1]*np.sin(curPos[2]);
    
    #Generated
    U[0] = 0.16*v[0] + 0.03*curVel[0]**3*np.sin(curPos[2]) + 0.04*curVel[1]**3*np.sin(curPos[2]) - 0.02*curVel[2]**3*np.sin(curPos[2]) + 0.06*v[0]*np.cos(2.0*curPos[1] + curPos[2]) + 0.03*v[0]*np.cos(2*curPos[1] + 2*curPos[2]) + 0.03*v[0]*np.cos(2.0*curPos[1]) + 0.09*curVel[0]**3*np.sin(2.0*curPos[1] + curPos[2]) + 0.06*v[0]*np.cos(curPos[2]) + 0.06*curVel[0]**3*np.sin(2*curPos[1] + 2*curPos[2]) + 0.03*curVel[0]**3*np.sin(2.0*curPos[1]) - 0.12*curVel[0]*curVel[1]**2*np.sin(2.0*curPos[1] + curPos[2]) - 0.03*curVel[0]**2*curVel[1]*np.sin(2.0*curPos[1] + curPos[2]) - 0.06*curVel[0]*curVel[2]**2*np.sin(2.0*curPos[1] + curPos[2]) + 0.03*curVel[0]**2*curVel[2]*np.sin(2.0*curPos[1] + curPos[2]) - 0.06*curVel[0]*curVel[1]**2*np.sin(2*curPos[1] + 2*curPos[2]) - 0.06*curVel[0]*curVel[2]**2*np.sin(2*curPos[1] + 2*curPos[2]) - 0.06*curVel[0]*curVel[1]**2*np.sin(2.0*curPos[1]) - 0.03*curVel[0]**2*curVel[1]*np.sin(2.0*curPos[1]) + 0.03*curVel[0]**2*curVel[2]*np.sin(2.0*curPos[1]) + 0.04*curVel[0]*curVel[1]**2*np.sin(curPos[2]) + 0.03*curVel[0]**2*curVel[1]*np.sin(curPos[2]) - 0.08*curVel[0]*curVel[2]**2*np.sin(curPos[2]) - 0.03*curVel[0]**2*curVel[2]*np.sin(curPos[2]) - 0.1*curVel[1]*curVel[2]**2*np.sin(curPos[2]) - 0.04*curVel[1]**2*curVel[2]*np.sin(curPos[2]) - 0.18*curVel[0]*curVel[1]*curVel[2]*np.sin(2.0*curPos[1] + curPos[2]) - 0.12*curVel[0]*curVel[1]*curVel[2]*np.sin(2*curPos[1] + 2*curPos[2]) - 0.06*curVel[0]*curVel[1]*curVel[2]*np.sin(2.0*curPos[1]) - 0.14*curVel[0]*curVel[1]*curVel[2]*np.sin(curPos[2])
    U[1] = 0.08*v[1] + 0.02*v[2] + 1.4715*np.cos(curPos[1]) + 0.03*curVel[0]**3*np.sin(curPos[2]) + 0.04*curVel[1]**3*np.sin(curPos[2]) - 0.02*curVel[2]**3*np.sin(curPos[2]) + 0.09*curVel[0]**3*np.sin(2.0*curPos[1] + curPos[2]) + 0.08*v[1]*np.cos(curPos[2]) + 0.02*v[2]*np.cos(curPos[2]) + 0.06*curVel[0]**3*np.sin(2*curPos[1] + 2*curPos[2]) + 0.03*curVel[0]**3*np.sin(2.0*curPos[1]) - 0.12*curVel[0]*curVel[1]**2*np.sin(2.0*curPos[1] + curPos[2]) - 0.03*curVel[0]**2*curVel[1]*np.sin(2.0*curPos[1] + curPos[2]) - 0.06*curVel[0]*curVel[2]**2*np.sin(2.0*curPos[1] + curPos[2]) + 0.03*curVel[0]**2*curVel[2]*np.sin(2.0*curPos[1] + curPos[2]) - 0.06*curVel[0]*curVel[1]**2*np.sin(2*curPos[1] + 2*curPos[2]) - 0.06*curVel[0]*curVel[2]**2*np.sin(2*curPos[1] + 2*curPos[2]) - 0.06*curVel[0]*curVel[1]**2*np.sin(2.0*curPos[1]) - 0.03*curVel[0]**2*curVel[1]*np.sin(2.0*curPos[1]) + 0.03*curVel[0]**2*curVel[2]*np.sin(2.0*curPos[1]) + 0.04*curVel[0]*curVel[1]**2*np.sin(curPos[2]) + 0.03*curVel[0]**2*curVel[1]*np.sin(curPos[2]) - 0.08*curVel[0]*curVel[2]**2*np.sin(curPos[2]) - 0.03*curVel[0]**2*curVel[2]*np.sin(curPos[2]) - 0.1*curVel[1]*curVel[2]**2*np.sin(curPos[2]) - 0.04*curVel[1]**2*curVel[2]*np.sin(curPos[2]) - 0.18*curVel[0]*curVel[1]*curVel[2]*np.sin(2.0*curPos[1] + curPos[2]) - 0.12*curVel[0]*curVel[1]*curVel[2]*np.sin(2*curPos[1] + 2*curPos[2]) - 0.06*curVel[0]*curVel[1]*curVel[2]*np.sin(2.0*curPos[1]) - 0.14*curVel[0]*curVel[1]*curVel[2]*np.sin(curPos[2])
    U[2] = 0.02*v[1] + 0.02*v[2] + 0.4905*np.cos(curPos[2]) + 0.03*curVel[0]**3*np.sin(curPos[2]) + 0.04*curVel[1]**3*np.sin(curPos[2]) - 0.02*curVel[2]**3*np.sin(curPos[2]) + 0.09*curVel[0]**3*np.sin(2.0*curPos[1] + curPos[2]) + 0.02*v[1]*np.cos(curPos[2]) + 0.06*curVel[0]**3*np.sin(2*curPos[1] + 2*curPos[2]) + 0.03*curVel[0]**3*np.sin(2.0*curPos[1]) - 0.12*curVel[0]*curVel[1]**2*np.sin(2.0*curPos[1] + curPos[2]) - 0.03*curVel[0]**2*curVel[1]*np.sin(2.0*curPos[1] + curPos[2]) - 0.06*curVel[0]*curVel[2]**2*np.sin(2.0*curPos[1] + curPos[2]) + 0.03*curVel[0]**2*curVel[2]*np.sin(2.0*curPos[1] + curPos[2]) - 0.06*curVel[0]*curVel[1]**2*np.sin(2*curPos[1] + 2*curPos[2]) - 0.06*curVel[0]*curVel[2]**2*np.sin(2*curPos[1] + 2*curPos[2]) - 0.06*curVel[0]*curVel[1]**2*np.sin(2.0*curPos[1]) - 0.03*curVel[0]**2*curVel[1]*np.sin(2.0*curPos[1]) + 0.03*curVel[0]**2*curVel[2]*np.sin(2.0*curPos[1]) + 0.04*curVel[0]*curVel[1]**2*np.sin(curPos[2]) + 0.03*curVel[0]**2*curVel[1]*np.sin(curPos[2]) - 0.08*curVel[0]*curVel[2]**2*np.sin(curPos[2]) - 0.03*curVel[0]**2*curVel[2]*np.sin(curPos[2]) - 0.1*curVel[1]*curVel[2]**2*np.sin(curPos[2]) - 0.04*curVel[1]**2*curVel[2]*np.sin(curPos[2]) - 0.18*curVel[0]*curVel[1]*curVel[2]*np.sin(2.0*curPos[1] + curPos[2]) - 0.12*curVel[0]*curVel[1]*curVel[2]*np.sin(2*curPos[1] + 2*curPos[2]) - 0.06*curVel[0]*curVel[1]*curVel[2]*np.sin(2.0*curPos[1]) - 0.14*curVel[0]*curVel[1]*curVel[2]*np.sin(curPos[2])
    
    return U,desiredThetas

#convets omegas (w) to skew matrix (w hat)
def omegaToSkew(omega):
     
    w = np.matrix([[0,-omega[2],omega[1]],
                   [omega[2],0,-omega[0]],
                   [-omega[1],omega[0],0]])

    return w

def expTransform(S,theta):
    w = omegaToSkew(S[0:3])
    v = S[3:6]

    R = np.eye(3) + (m.sin(theta)*w) + ((1-m.cos(theta))*w**2)
    R = np.append(R,np.array([[0,0,0]]),axis=0)
    V = np.matmul((np.eye(3)*theta+(1-m.cos(theta))*w + (theta-m.sin(theta)*w**2)),v).transpose()
    V = np.append(V,np.array([[1]]),axis=0)

    ES = np.concatenate((R,V),axis=1)
    return ES

def fkWorld(M,Slist,thetaList):
    T = M
    for i in range(thetaList.shape[0]-1,-1,-1):
        T = np.matmul(expTransform(Slist[:,i],thetaList[i]),T)
    return T

if __name__ == '__main__':
    np.set_printoptions(precision=3,suppress=True)

    M = np.array([[1,0,0,870],
                  [0,1,0,0],
                  [0,0,1,1195],
                  [0,0,0,1]])
    thetaList = np.array([0,0,0])
    SList = np.array([[0,0,0],[0,-1,-1],[1,0,0],[0,475,1075],[0,0,0],[0,-150,-150]])
    T03 = fkWorld(M,SList,thetaList)
    print(T03)
    targetLog,thetaLog = main()


    fig, ax = plt.subplots()
    ax.plot(targetLog,label='target')
    ax.plot(thetaLog,label='actual')
    ax.set_title("Target/Actual Angular Position over time")
    ax.set_xlabel('Simulation Steps (240hz)')
    ax.set_ylabel('Angle (rads)')
    ax.legend()
    plt.show()