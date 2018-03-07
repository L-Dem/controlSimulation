import random
import math
import numpy


class Particle:
    """"distributed estimation and control"""
    def __init__(self, count, position):
        self.id = count  # 一共有多少个粒子
        self.desiredMoment = position
        self.count_phi = len(position)
        self.positionX = random.uniform(3, 5)  # 粒子位置
        self.positionY = random.uniform(3, 5)
        self.a = 0  # 平均一致p参数
        self.b = 0  # 平均一致i参数
        self.omega = numpy.zeros(self.count_phi)  # 内部状态估计器
        self.omega_d = numpy.zeros(self.count_phi)  #
        self.x = numpy.zeros(self.count_phi)  # 状态估计器
        self.x_d = numpy.zeros(self.count_phi)
        self.r = 0.4
        self.B = 0  # 控制律参数
        self.gamma = numpy.diag(numpy.ones(self.count_phi))  # 控制律参数[80, 80, 8, 8, 8]
        self.phi = numpy.zeros(self.count_phi)  # phi矩阵为1*5
        self.phi_d = numpy.zeros((2, self.count_phi))  # 粒子的矩的雅可比矩阵。应该为2*5
        self.compute_phi()  # 粒子当前的矩[a, b, a^2, ab, b^2]
        self.velocityX = 1  # 粒子速度
        self.velocityY = 1
        self.accelerationX = 1  # 粒子加速度
        self.accelerationY = 1
        self.distance = []  # 该粒子与其他粒子的距离,矩阵
        self.radius = 20  # 通讯半径
        self.neighbour = []  # 该粒子的neighbour
        self.dt = 0.01

    ''''PI估计器'''
    def estimation_pi(self):  # unfinished
        self.r = 10
        middle1 = numpy.zeros(self.count_phi)  # numpy.array([0., 0, 0, 0, 0])
        middle2 = numpy.zeros(self.count_phi)  # numpy.array([0, 0, 0, 0, 0])
        middle3 = numpy.zeros(self.count_phi)  # numpy.array([0, 0, 0, 0, 0])
        for j in self.neighbour:
            self.a = 4
            self.b = 0.2
            middle1 = middle1 + self.a * (self.x - j.x)
            middle2 = middle2 + self.b * (self.omega - j.omega)
            middle3 = middle3 + self.b * (self.x - j.x)
        self.omega_d = -middle3
        self.omega = self.omega_d * self.dt + self.omega
        self.x_d = - self.r * self.x - middle1 + middle2 + self.r * self.phi
        self.x = self.x + self.x_d * self.dt

    ''''P估计器'''
    def estimation_p(self):
        self.r = 0  # r = 0.7
        middle = numpy.zeros(self.count_phi)  # numpy.array([0., 0, 0, 0, 0])
        for j in self.neighbour:
            self.a = 0.3
            middle = middle + self.a * (self.x - j.x)
        self.omega_d = -self.r * self.omega - middle
        self.omega = self.omega + self.omega_d * self.dt
        self.x = self.omega + self.phi

    ''''P控制律'''
    def control_run_p(self):
        self.B = 10  # 下面计算公式分成两个部分
        middle = numpy.dot(self.phi_d, self.gamma)
        middle = numpy.dot(middle, self.desiredMoment.T - self.x.T)
        self.accelerationX = (-self.B * self.velocityX + middle[0])
        self.accelerationY = (-self.B * self.velocityY + middle[1])
        self.velocityX = self.velocityX + self.accelerationX * self.dt
        self.velocityY = self.velocityY + self.accelerationY * self.dt
        self.positionX = self.positionX + self.velocityX * self.dt
        self.positionY = self.positionY + self.velocityY * self.dt

    ''''PI控制律'''
    def control_run_pi(self):
        self.B = 3  # 下面计算公式分成两个部分
        middle = numpy.dot(self.phi_d, self.gamma)
        middle = numpy.dot(middle, self.desiredMoment.T - self.x.T)
        self.accelerationX = -self.B * self.velocityX + middle[0]
        self.accelerationY = -self.B * self.velocityY + middle[1]
        self.velocityX = self.velocityX + self.accelerationX * self.dt
        self.velocityY = self.velocityY + self.accelerationY * self.dt
        self.positionX = self.positionX + self.velocityX * self.dt
        self.positionY = self.positionY + self.velocityY * self.dt

    ''''计算单一智能体的phi和phi的雅可比矩阵'''
    def compute_phi_old(self):  # 雅可比矩阵的计算，使用不同x，y的偏导
        self.phi = [
            self.positionX,
            self.positionY,
            math.pow(self.positionX, 2),
            self.positionX * self.positionY,
            math.pow(self.positionY, 2),
        ]

        self.phi_d = [
            [1, 0, 2 * self.positionX, self.positionY, 0],
            [0, 1, 0, self.positionX, 2 * self.positionY]
        ]

    ''''寻找当前智能体的neighbour'''
    def find_neighbour(self, particle_list):
        self.neighbour = []
        for par in particle_list:
            distance_x = par.positionX - self.positionX
            distance_y = par.positionY - self.positionY
            self.distance = math.sqrt(math.pow(distance_x, 2) + math.pow(distance_y, 2))
            if self.distance != 0 and self.distance < 10:
                self.neighbour.append(par)

    '''phi计算的封装'''
    def compute_phi(self):
        count = 0
        self.phi = numpy.zeros(self.count_phi)
        self.phi_d = numpy.zeros((2, self.count_phi))
        while True:
            for k in range(1, 10):  # 最高可以取到十阶矩
                j = 0
                while j <= k:  # 每阶按照position_x的降序排列
                    self.phi[count] = math.pow(self.positionX, (k - j)) * math.pow(self.positionY, j)
                    self.phi_d[0][count] = (k - j) * math.pow(self.positionX, (k - j - 1)) * math.pow(self.positionY, j)
                    self.phi_d[1][count] = j * math.pow(self.positionY, (j - 1)) * math.pow(self.positionX, (k - j))
                    count = count + 1
                    j = j + 1
                    if count == self.count_phi:
                        break
                if count == self.count_phi:
                    break
            if count == self.count_phi:
                break

    def compute_phi_high(self):  # moment > 5的情况
        count = 0
        self.phi = numpy.zeros(self.count_phi)
        self.phi_d = numpy.zeros((2, self.count_phi))
        while True:
            for k in range(1, 3):  # 前两阶按次序取值
                j = 0
                while j <= k:  # 每阶只取最低次和最高次
                    self.phi[count] = math.pow(self.positionX, (k - j)) * math.pow(self.positionY, j)
                    self.phi_d[0][count] = (k - j) * math.pow(self.positionX, (k - j - 1)) * math.pow(self.positionY, j)
                    self.phi_d[1][count] = j * math.pow(self.positionY, (j - 1)) * math.pow(self.positionX, (k - j))
                    count = count + 1
                    j = j + 1

            for k in range(3, 10):  # 三阶以上
                j = 0
                while j <= k:  # 每阶只取最低次和最高次
                    self.phi[count] = math.pow(self.positionX, (k - j)) * math.pow(self.positionY, j)
                    self.phi_d[0][count] = (k - j) * math.pow(self.positionX, (k - j - 1)) * math.pow(self.positionY, j)
                    self.phi_d[1][count] = j * math.pow(self.positionY, (j - 1)) * math.pow(self.positionX, (k - j))
                    count = count + 1
                    j = k
                    if count == self.count_phi:
                        break
                if count == self.count_phi:
                    break
            if count == self.count_phi:
                break
