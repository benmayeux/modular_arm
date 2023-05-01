import numpy as np
import time
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

if __name__=="__main__":
    desiredPos = [1.57,-.785,-.785]
    staringPos = [0,0,0]
    start_time = time.time()
    print(start_time)
    print(time.time()-start_time)
    while(time.time()-start_time < 5 ):
        currT = time.time()-start_time
        desiredThetas = traj_evaluate(0,5,currT,[0,0,0],desiredPos)
        print(desiredThetas)
