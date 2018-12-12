import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from numpy import linalg as LA


class GuidanceVectorFieldEllipse:
    def __init__(self, a,b, x0, y0):
        self.a = a  # length of ellipse
        self.b = b  # width of ellipse 
        self.x0 = x0 # center of ellipse in x direction
        self.y0 = y0 # center of ellipse in y direction

    def computeNormal(self, pos_xy):
        # f(p) = ((x-pos_xy[0])/self.a)**2 + ((y-pos_xy)/self.b)**2 - 1

        normal = np.array([2.* (pos_xy[0] - self.x0)/self.a**2, 2.* (pos_xy[1] - self.y0)/self.b**2]) 
        

        return normal

    def computeTangent(self,normal):
        E = np.array([[0., 1.],[-1., 0.]])

        return E.dot(normal)


    def computeError(self, pos_xy):
        totalError = ((pos_xy[0]-self.x0)/self.a)**2 + ((pos_xy[1]-self.y0)/self.b)**2 - 1.

        return totalError

    def createSingleVectorFieldVector(self, tangent, normal, totalError, k):
        boundary = 1.

        if abs(totalError) < boundary:
            error = 0
        elif totalError >= boundary: # outside ellipse
            error = 1
        else:   # inside
            error = -1

        #print("error", error)

        error = totalError

        return np.subtract(tangent, normal.dot(k*error))


    def getDesiredVelocity(self, pos_xy):
        normal = self.computeNormal(pos_xy)
        tangent = self.computeTangent(normal)
        totalError = self.computeError(pos_xy)

        singleVectorFieldVector = self.createSingleVectorFieldVector(tangent, normal, totalError, 0.3)
        
        singleVectorFieldVector = singleVectorFieldVector.dot(1./LA.norm(singleVectorFieldVector))
        return singleVectorFieldVector
        


def plotVectorField():
    I = np.array([])
    J = np.array([])
    Px = np.array([])
    Py = np.array([])
    n = 0

    gvf = GuidanceVectorFieldEllipse(4,6,0,0) 

    for i in range(-10, 11):
        for j in range(-10,11):
            #print(i)


            pos = (i,j)
            normal = gvf.computeNormal(pos)
            tangent = gvf.computeTangent(normal)
            totalError = gvf.computeError(pos)

            singleVectorFieldVector = gvf.createSingleVectorFieldVector(tangent, normal, totalError, 0.3)
            
            singleVectorFieldVector = singleVectorFieldVector * 1./LA.norm(singleVectorFieldVector)


            I = np.concatenate([I, [i]])
            J = np.concatenate([J, [j]])
            Px = np.concatenate([Px, [singleVectorFieldVector[0]]])
            Py = np.concatenate([Py, [singleVectorFieldVector[1]]])
            
            
            n +=1

    #plt.axes([-10, 0.025, 0.95, 0.95])
    
    #plt.quiver(I, J, Px, Py, alpha=1.)
    #plt.quiver(I, J, Px, Py, edgecolor='k', facecolor='None', linewidth=.5)
    #plt.axis('equal')
    e1 = patches.Ellipse((gvf.x0, gvf.y0), gvf.a*2., gvf.b*2., linewidth=2, fill=False)
    
    #plt.xlim(-1, n)
    #plt.xticks(())
    #plt.ylim(-1, n)
    #plt.yticks(())

    



    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.add_patch(e1)

    ax.quiver(I, J, Px, Py, alpha=1.)
    ax.quiver(I, J, Px, Py, edgecolor='k', facecolor='None', linewidth=.5)
    ax.axis('equal')

    ax.set_xlim(-10,10)
    ax.set_ylim(-10,10)

    plt.show()
    

#plotVectorField()


# pos = (5,0)
# gvf = GuidanceVectorFieldEllipse(2,3,0,0) 
# normal = gvf.computeNormal(pos)
# tangent = gvf.computeTangent(normal)
# totalError = gvf.computeError(pos)

# singleVectorFieldVector = gvf.createSingleVectorFieldVector(tangent, normal, totalError, 2.)

# print("Position:", pos)
# print("normal:", normal)
# print("tangent:", tangent)
# print("totalError:", totalError)
# print("singleVectorFieldVector:", singleVectorFieldVector)
