from scipy import linalg as la
import matplotlib.pyplot as pl
import matplotlib.mlab as mlab
import numpy as np




class kalman:
    def __init__(self, dt, positionEstimate):
        self.dt = dt # velocity update

        # Set initial conditions for the Kalman Filter
        #self.X_hat = np.random.uniform(-10, 10, 2) # Random 2x1 vector from -10 to 10
        
        self.X_hat = positionEstimate
        self.P = np.array([[5., 0.],[0., 5.]])

        # Standard deviations for the measurements (we consider them constant for the whole simulation)
        self.sigma_d = 0.2 # distance measurement standard deviation
        self.sigma_rel_vel = np.array([0.001,0.001]) # relative velocity uncertenty in each direction (xyz)

        # Data log
        #self.X_hat_log = np.zeros((time.size, X_hat.size))
        #self.P_log = np.zeros((time.size, P.size))


        self.F = np.array([[1., 0.], [0., 1.]])

        self.G = np.array([[self.dt, 0.], [0., self.dt]])

        self.Q = np.outer(self.G.dot(self.sigma_rel_vel), self.sigma_rel_vel.transpose().dot(self.G.transpose())) # process noise


    # propagation
    def performPredictionStep(self, rel_vel_mes):
        u = rel_vel_mes[:2]

        #print("HELLLO")
        #print(self.X_hat)
        #print(self.F)
        #print(self.G)
        #print(u)
        

        self.X_hat = self.F.dot(self.X_hat)+self.G.dot(u)

        #print(self.X_hat , "xhat")

        #print("Helloo")
        #print(self.Q)
        #print(self.F)
        #print(self.F.transpose())
        #print(self.P)
        #print(self.G)
        self.P = self.F.dot(self.P).dot(self.F.transpose()) + self.Q

    # correction
    def performCorrectionStep(self, dis_mes):
        H = (1./la.norm(self.X_hat)) * np.array([self.X_hat[0], self.X_hat[1]])

        

        R = (H*self.sigma_d).dot(self.sigma_d*H.transpose())

        K = self.P.dot(H.transpose()) * (1./(H.dot(self.P.dot(H.transpose())) + R)) 
        
        #print(dis_mes - la.norm(self.X_hat))
        #print(dis_mes, "Distance measure")
        #print(la.norm(self.X_hat))

        #print("Jet jet et los")
        #print("H:", H)
        #print("R:", R)
        #print("K: ", K)
        #print("Stuff1", self.P.dot(H.transpose()))
        #print("Stuff2: ", (1./(H.dot(self.P.dot(H.transpose())) + R)) )
        #print("P:", self.P)

        

        #print(self.X_hat, "before")
        

        self.X_hat = self.X_hat + K * (dis_mes - la.norm(self.X_hat))

        #print(self.X_hat, "after")
        #print(self.P, " P before")
        self.P = self.P - np.outer(K, H.dot(self.P))
        #print(self.P, " P after")
        #H = # Eq. (4)
        #R = 
        #K = # Eq. (5)

        #e = la.norm(p) - la.norm(X) # e = r - h(\hat x) in Eq. (6)
        #X_hat_u = # Eq. (6)
        #P_u = # Eq. (7)

        #X_hat = X_hat_u
        #P = P_u

        pass
