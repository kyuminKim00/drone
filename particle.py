import numpy as np
import signal
import sys
import rospy
from geometry_msgs.msg import Point

class Particle:
    def __init__(self, x, y, z, weight):
        self.x = x
        self.y = y
        self.z = z
        self.weight = weight


class ParticleFilter:
    def __init__(self, num_particles):
        self.num_particles = num_particles
        self.particles = []

    def initialize(self, initial_state):
        self.particles = []
        for _ in range(self.num_particles):
            x = initial_state[0]  
            y = initial_state[1] 
            z = initial_state[2] 
            weight = 1.0 / self.num_particles  # 모든 입자의 가중치 초기화
            particle = Particle(x, y, z, weight)
            self.particles.append(particle)

    def predict(self,pose_arr):
        for particle in self.particles:
            dx = pose_arr[0]
            dy = pose_arr[1]
            dz = pose_arr[2] 
            # particle.x += dx*0.1
            # particle.y += dy*0.1
            # particle.z += dz*0.1
            particle.x = dx
            particle.y = dy
            particle.z = dz

    def update(self, measurement):
        total_weight = 0.0
        for particle in self.particles:
            # 측정값과 입자 위치 사이의 유사도 계산
            particle.weight = self.compute_similarity(measurement, particle.x, particle.y, particle.z)
            total_weight += particle.weight

        # 입자의 가중치를 정규화
        for particle in self.particles:
            particle.weight /= total_weight

    def resample(self):
        weights = [particle.weight for particle in self.particles]
        indices = np.random.choice(range(self.num_particles), size=self.num_particles, replace=True, p=weights)

        resampled_particles = []
        for index in indices:
            resampled_particles.append(self.particles[index])

        self.particles = resampled_particles

    def compute_similarity(self, measurement, x, y, z):
        # 측정값과 예측값 사이의 유사도 계산 (예시에서는 간단히 유클리드 거리 사용)

        return 1.0 / np.sqrt((x - 0.95*measurement[0]) ** 2 + (y - 0.95*measurement[1]) ** 2 + (z - 0.95*measurement[2]) ** 2)
    
class listen():
    def __init__(self) :
        self.sub = rospy.Subscriber("pose_UWB",Point,self.callback)
        rospy.init_node('test',anonymous=False)

        self.x = 0
        self.y = 0
        self.z = 0
    def callback(self,msg):
        self.x = msg.x
        self.y = msg.y
        self.z = msg.z



if __name__ == '__main__':
    def force_exit(_,__):
        print('\nCtrl+C->exit')
        sys.exit(0)
    signal.signal(signal.SIGINT ,force_exit)

   

    lis = listen()
    num_particles = 10000


    initial_state = (0.0, 0.0, 0.0)
    filter = ParticleFilter(num_particles)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():

        measurement = [lis.x,lis.y,lis.z]
        motion_model = measurement
 #       filter = ParticleFilter(num_particles)
        filter.initialize(initial_state)
        filter.predict(motion_model)  # 모션 모델을 적용하여 입자의 예측 위치 계산
        filter.update(measurement)  # 입자의 가중치를 측정값과 비교하여 업데이트
        filter.resample()  # 가중치를 기반으로 입자를 재샘플링

       
        estimated_position = np.mean([(particle.x, particle.y, particle.z) for particle in filter.particles])

        initial_state = estimated_position
        rate.sleep()
   # print("Estimated position:", estimated_position)

