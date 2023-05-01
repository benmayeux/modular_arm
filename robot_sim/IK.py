import numpy as np


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