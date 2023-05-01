from statistics import mode
import numpy as np

#convets omegas (w) to skew matrix (w hat)
def omegaToSkew(omega):
     
    w = np.matrix([[0,-omega[2],omega[1]],
                   [omega[2],0,-omega[0]],
                   [-omega[1],omega[0],0]])

    return w

def expTransform(S,theta):
    w = omegaToSkew(S[0:3])
    v = S[3:6]

    R = np.eye(3) + (np.sin(theta)*w) + ((1-np.cos(theta))*w**2)
    R = np.append(R,np.array([[0,0,0]]),axis=0)
    V = np.matmul((np.eye(3)*theta+(1-np.cos(theta))*w + (theta-np.sin(theta)*w**2)),v).transpose()
    V = np.append(V,np.array([[1]]),axis=0)

    ES = np.concatenate((R,V),axis=1)
    return ES

def fkWorld(M,Slist,thetaList):
    T = M
    for i in range(thetaList.shape[0]-1,-1,-1):
        T = np.matmul(expTransform(Slist[:,i],thetaList[i]),T)
    return T

def calcFK(modelType,thetaList):
    lengthOfRLinks = 0.211201 # in m
    baseHOffset = 0.049276
    baseVOffset = 0.2014855
    if modelType == 'RRR':
        M = np.array([[1,0,0,0],
                    [0,1,0,2*lengthOfRLinks+2*baseHOffset],
                    [0,0,1,baseVOffset],
                    [0,0,0,1]])
        print('M: \n', M)
        Q = np.array([[0,0,0],[0,lengthOfRLinks+baseHOffset,baseVOffset],[0,2*lengthOfRLinks+2*baseHOffset,baseVOffset]])
        W = np.array([[0,0,1],[-1,0,0],[-1,0,0]])
        SList = np.zeros((6,thetaList.shape[0]))
        
        for i,(wi,qi) in enumerate(zip(W,Q)):
            vi = np.cross(-wi,qi)
            SList[:,i] = np.concatenate((wi,vi))
        
        T03 = fkWorld(M,SList,thetaList)
        print("SList: \n", SList)
        print('Transform at given angles: \n ', T03)

def debugTest():
    lengthOfRLinks = 0.211201 # in m
    baseHOffset = 0.049276
    baseVOffset = 0.2014855
    np.set_printoptions(precision=3,suppress=True)
    M = np.array([[1,0,0,2*lengthOfRLinks+2*baseHOffset],
                  [0,1,0,0],
                  [0,0,1,baseVOffset],
                  [0,0,0,1]])
    thetaList = np.array([1.57,0,0])
    SList = np.array([[0,0,0],[0,-1,-1],[1,0,0],[0,475,1075],[0,0,0],[0,-150,-150]])
    T03 = fkWorld(M,SList,thetaList)
    print("T03: \n ",SList)

if __name__ == '__main__':
    thetaList = np.array([1.57,0,0])
    calcFK('RRR',thetaList)