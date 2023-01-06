import numpy as np
class Controller:
    """Linear controller for the steering wheel""" 
    def __init__(self,qsi: float,w_n: float,v_ref: float,w_ref: float,h: float,L: float):
        """
        Parameters
        ----------
        qsi: float
            Damping Ratio
        w_n: float
            Natural Frequency
        v_ref: float
            Linear velocity reference (km/h)
        w_ref: float 
            Steering wheel velocity reference 
        h: float
            Integration Step
        L: float
            Car dimension
        """

        self.qsi = qsi
        self.w_n = w_n
        self.v_ref = v_ref/3.6 #m/s
        self.w_ref = w_ref
        self.kv= 2*qsi*w_n
        self.ks= self.kv
        self.ki = (w_n**2 - w_ref**2)/abs(self.v_ref)
        self.h = h
        self.L = L 
    
    def print_parameters(self):
        """
            Print parameters of the controller
        """
        print(
            f"""

                qsi value: {self.qsi}
                w_n value: {self.w_n}
                v_ref value: {self.v_ref} m/s
                w_ref value: {self.w_ref}
                kv value: {self.kv}
                ks value: {self.ks}
                ki value: {self.ki}
                h value: {self.h}
                L value: {self.L}
                Closed Loop poles of the system {self.poles()}

            """


        )
    def following_trajectory(self,ref: np.array,position: np.array) -> np.array:
        """
            Computes next position and control signal given the current position and the next point using the linear controller

            Inputs: 
                -ref: a numpy array with dimensions 3x1 (x_ref,y_ref,theta_ref) of planned trajectory
                -position : an array with with dimensions 3x1 (x,y,theta) with estimated positions
            Outputs:
                -position : next position
                -np.array([v,ws]): control signal
        """
        world_error = ref-position
        bot_error = np.matmul(np.array([[np.cos(position[2,1]), np.sin(position[2,1]), 0 ], [-np.sin(position[2,1]), np.cos(position[2,1]), 0 ], [0 ,0 ,1 ]]),world_error)
        v = self.kv * bot_error[0]
        ws = self.ki * bot_error[1] + self.ks * bot_error[2]
        derivative = np.array([[np.cos(position[2,1]) ,0 ],[np.sin(position[2,1]), 0 ],[np.tan(position[3,1])/self.L, 0], [0, 1]])
        position = position + self.h*np.matmul(derivative, np.array([v,ws]))
        if abs(position[3,1]) > np.pi/8 :
            position[3,1] = np.sign(position[3,1])* np.pi/8
        return position,np.array([v,ws])
    pass

    def following_reference(self,ref: np.array,num_points: int= None) -> np.array:
        """
            Gives trajectory followed by the model using this kind of Controller

            Inputs: 
                -ref: a numpy array with dimensions 3xK (x_ref,y_ref,theta_ref) where K is the number of points to compute
                -num_points : a integer that expresses the number of points of trajectory vector (time_simulation*h)
            Outputs:
                -trajectory: a numpy array with dimensions 4xK (x,y,theta,phi) followed by the car model
        """
        if num_points != None: trajectory = np.zeros((ref.shape[0]+1, min(ref.shape[1],num_points)))
        else: trajectory =  np.zeros((ref.shape[0]+1, ref.shape[1]))


        for k in range(trajectory.shape[1]-1):
            world_error = ref[:,k] - trajectory[:-1,k]
            bot_error = np.matmul(np.array([[np.cos(trajectory[2,k]), np.sin(trajectory[2,k]), 0 ], [-np.sin(trajectory[2,k]), np.cos(trajectory[2,k]), 0 ], [0 ,0 ,1 ]]),world_error)
            v = self.kv * bot_error[0]
            ws = self.ki * bot_error[1] + self.ks * bot_error[2]
            derivative = np.array([[np.cos(trajectory[2,k]) ,0 ],[np.sin(trajectory[2,k]), 0 ],[np.tan(trajectory[3,k])/self.L, 0], [0, 1]])
            trajectory[:,k+1] = trajectory[:,k]+ self.h*np.matmul(derivative, np.array([v,ws]))
            if abs(trajectory[3,k+1]) > np.pi/8 :
                trajectory[3,k+1] = np.sign(trajectory[3,k+1])* np.pi/8
        return trajectory

    def poles(self):
        """
            Compute closed loop poles of the system
        """
        poles,_ = np.linalg.eig(np.array([[0, self.w_ref, 0], [-self.w_ref, 0, self.v_ref], [0, 0, 0]]) - np.matmul(np.array([[1 ,0 ], [0 ,0], [0, 1]]), np.array([[self.kv ,0 ,0],[0,self.ki,self.ks]])))
        return poles

if __name__ == "__main__":
    from matplotlib import pyplot as plt 
    from scipy import signal
    x_ref = np.arange(0,50,0.01)

    #Examples of trajectories
    y_ref_traj = [np.cos(0.02*x_ref),signal.square(2*np.pi*0.02*x_ref),np.zeros(x_ref.shape)+1]

    t_simulation = 10
    L = 2.2
    h = 0.01
    qsi = 1
    w_n = 10
    v_ref = 36 #km/h
    w_ref = 4
    num_points = int(t_simulation/h)
    simulation = Controller(qsi = qsi,w_n = w_n,v_ref=v_ref,w_ref = w_ref,h = h, L = L)
    simulation.print_parameters()
    for y_ref in y_ref_traj:
        teta_ref = np.zeros(y_ref.shape)
        for k in range((y_ref.shape[0])-1):
            teta_ref[k] = np.arctan2(y_ref[k+1] - y_ref[k], x_ref[k+1]-x_ref[k])

        Ref = np.array([ [x_ref],[y_ref],[teta_ref]])
        Ref = Ref.reshape(Ref.shape[0],Ref.shape[2])

        trajectory = simulation.following_reference(ref=Ref,num_points= None)
        plt.figure()
        plt.xlabel("x axis caption") 
        plt.ylabel("y axis caption") 
        plt.plot(trajectory[0,:],trajectory[1,:]) 
        plt.plot(Ref[0,:],Ref[1,:]) 

        plt.figure()
        plt.plot(trajectory[2,:]) 

    plt.show()
