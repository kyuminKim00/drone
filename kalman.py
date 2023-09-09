import numpy as np
import rospy
import numpy.linalg as lin
import time

dt=0.001

class KalmanFilter_3D:
    def __init__(self,datahub):
        super(KalmanFilter_3D, self).__init__()
        self.datahub = datahub
        
        ## state vector

            ## x = [x,y,z,vx,vy,vz].T

    #    # self.pose_arr_UWB = pose_arr_UWB
    #     self.pose_arr_CAM = pose_arr_CAM
    #     self.measure_UWB = np.array([self.pose_arr_UWB[0],self.pose_arr_UWB[1],self.pose_arr_UWB[2],0,0,0])
        
    #     #self.measure_UWB = np.diag([self.pose_arr_UWB[0],self.pose_arr_UWB[1],self.pose_arr_UWB[2],0,0,0])

    #     self.vel_vector = np.array([vel_ned[0],vel_ned[1],vel_ned[2]])

    #     self.pose_arr_combine = 0.4*self.pose_arr_UWB + 0.6*self.pose_arr_CAM
        
    #     self.state_vector_UWB = np.hstack((self.pose_arr_UWB,self.vel_vector)).T
    #     self.state_vector_combine = np.hstack((self.pose_arr_combine,self.vel_vector)).T
        
    #     self.measure_combine = np.array([self.pose_arr_combine[0],self.pose_arr_combine[1],self.pose_arr_combine[2],0,0,0])
        


        self.inInitialized = False
        self.inInitialized_com = False
        self.A = np.eye(3)
        # self.A[0][3] = delt
        # self.A[1][4] = delt
        # self.A[2][5] = delt
        self.H = np.eye(3)
        # self.H[3][3] = 0
        # self.H[4][4] = 0
        # self.H[5][5] = 0
        # ## param
        #self.Q = processNoise


        # # self.R_UWB= np.diag([0.02,0.02,0.02,0,0,0])
        # # self.R_CAM = np.diag([0.006,0.006,0.006,0,0,0])       
        # # self.Weight_UWB = 0.4
        # self.Weight_CAM = 0.6

        # self.R_combined = self.Weight_CAM*self.R_UWB + self.Weight_UWB*self.R_CAM
        # #self.x_esti = 0
        # self.p = 0.0
        self.x_pred = 0.0
        self.p_pred = 0.0
        self.p = 0.0
        self.K_gain = 0.0
        self.x_pred_com = 0.0
        self.p_pred_com = 0.0
        self.p_com = 0.0
        self.K_gain_com = 0.0
        self.x_esti_UWB = 0
        self.x_esti_CAM = 0

        self.B = 0.1*np.eye(3)
    def filtering_3D_UWB(self,pose_UWB,control_input):
            ## system noise
            Q = 0.001*np.eye(3)
            R = 0.009*np.eye(3)

            if not self.inInitialized :
                self.inInitialized = True
                self.x_pred =self.A @ pose_UWB + self.B @ control_input
                self.p_pred = np.eye(3)
            else:
                self.x_pred = self.x_esti_UWB
                self.p_pred = self.A.dot(self.p).dot(self.A.T)+Q
            
            self.K_gain = self.p_pred @ self.H.T @ lin.pinv(self.H @ self.p_pred @ self.H.T +R)

            self.x_esti_UWB = self.x_pred + self.K_gain.dot(pose_UWB - self.H @ self.x_pred)

            p= self.p_pred-self.K_gain.dot(self.H).dot(self.p_pred)
 
            
            return self.x_esti_UWB , p
    
    def filtering_3D_COM(self,pose_CAM , control_input):
        ## noise
        Q = 0.001*np.eye(3)
        R = 0.001*np.eye(3)
        if pose_CAM.all() == None :
            if not self.inInitialized_com :
                    self.inInitialized_com = True
                    self.x_pred_com =self.A @ pose_CAM + self.B @ control_input
                    self.p_pred_com = np.eye(3)
                    
            else:
                
                self.x_pred_com = self.x_esti_CAM
                self.p_pred_com = self.A.dot(self.p_com).dot(self.A.T)+Q        

            self.K_gain_com = self.p_pred_com @ self.H.T @ lin.pinv(self.H @ self.p_pred_com @ self.H.T +R)
        
            self.x_esti_CAM = self.x_pred_com + self.K_gain_com.dot((pose_CAM) - self.H @ self.x_pred_com)
        
            self.p_com= self.p_pred_com-self.K_gain_com.dot(self.H).dot(self.p_pred_com)
            #print(self.x_esti)
        else : 
            self.p_com = np.eye(3)
            self.x_esti_CAM = np.array([0,0,0])
        return self.x_esti_CAM , self.p_com
         
    
class KalmanFilter_1D():
	def __init__(self, processNoise = 0.1, measurementNoise = 0.2 ):
		super(KalmanFilter_1D, self).__init__()
		self.isInitialized = False
		self.processNoise = processNoise
		self.measurementNoise = measurementNoise
		self.predictedRSSI = 0
		self.errorCovariance = 0

	def filtering(self, rssi):
		if not self.isInitialized:
			self.isInitialized = True
			priorRSSI = rssi
			priorErrorCovariance = 1
		else:
			priorRSSI = self.predictedRSSI
			priorErrorCovariance = self.errorCovariance + self.processNoise

		kalmanGain = priorErrorCovariance / (priorErrorCovariance + self.measurementNoise)
		self.predictedRSSI = priorRSSI + (kalmanGain * (rssi - priorRSSI))
		self.errorCovarianceRSSI = (1 - kalmanGain) * priorErrorCovariance
        
		return self.predictedRSSI

if __name__ == '__main__':
      k=Kalmanfilter_3D()
      CAM_pose_arr=None
      while True:  
        UWB_pose_arr=np.array([1,1,1])
        CAM_pose_arr=np.array([1.2,1.2,1.2])
        a= k.filtering_3D(UWB_pose_arr,CAM_pose_arr)
        time.sleep(1)
        print(a)