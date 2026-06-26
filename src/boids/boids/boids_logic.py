import numpy as np

def step(state, p):
    xVec = state['x']
    yVec = state['y']
    vxVec = state['vx']
    vyVec = state['vy']
    N = xVec.shape[0]

    for i in range(N):
        idx = [a for a in range(N) if a != i]
        xi = xVec[i]
        yi = yVec[i]
        vxi = vxVec[i]
        vyi = vyVec[i]
        xAll = xVec[idx]
        yAll = yVec[idx]
        vxAll = vxVec[idx]
        vyAll = vyVec[idx]
        dx = xi - xAll
        dy = yi - yAll
        D = np.sqrt(dx**2 + dy**2)

        idVisual = [a for a in range(len(D)) if D[a] <= p['VR']]
        idProtected = [a for a in range(len(D)) if D[a] <= p['PR']]
        closeDX = sum(dx[idProtected])
        closeDY = sum(dy[idProtected])
        neighboringBoids = len(idVisual)
        if neighboringBoids > 0:
            xPosAvg = sum(xAll[idVisual]) / neighboringBoids
            yPosAvg = sum(yAll[idVisual]) / neighboringBoids
            xVelAvg = sum(vxAll[idVisual]) / neighboringBoids
            yVelAvg = sum(vyAll[idVisual]) / neighboringBoids
        else:
            xPosAvg = 0
            yPosAvg = 0
            xVelAvg = 0
            yVelAvg = 0

        # 3 Cohesion
        vxCohesion = (xPosAvg - xi) * p['CF']
        vyCohesion = (yPosAvg - yi) * p['CF']

        # 1 Separation
        vxSeparation = closeDX * p['SF']
        vySeparation = closeDY * p['SF']

        # 2 Alignment
        vxAlignment = (xVelAvg - vxi) * p['AF']
        vyAlignment = (yVelAvg - vyi) * p['AF']

        vxi = vxi + vxSeparation + vxAlignment + vxCohesion
        vyi = vyi + vySeparation + vyAlignment + vyCohesion

        # 4 Boundary
        if xi <= p['x_safe_min']:
            vxi = vxi + p['TF']
        elif xi >= p['x_safe_max']:
            vxi = vxi - p['TF']
        if yi <= p['y_safe_min']:
            vyi = vyi + p['TF']
        elif yi >= p['y_safe_max']:
            vyi = vyi - p['TF']

        # 5 Speed normalization
        speedI2 = np.sqrt(vxi**2 + vyi**2)
        if speedI2 > p['max_speed']:
            vxi = (vxi / speedI2) * p['max_speed']
            vyi = (vyi / speedI2) * p['max_speed']
        elif speedI2 < p['min_speed']:
            vxi = (vxi / speedI2) * p['min_speed']
            vyi = (vyi / speedI2) * p['min_speed']

        # 6 Update positions
        xVec[i] = xi + vxi
        yVec[i] = yi + vyi
        vxVec[i] = vxi
        vyVec[i] = vyi

    return state