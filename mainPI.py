import particle
import numpy
import matplotlib.pyplot as plt

# if __name__ == '__main__':
element = []
count = 10  # 创建的粒子数

desiredMoment = numpy.array([0, 0, 5.0, 0, 5, 0])  # 目的位置
moment = numpy.zeros(len(desiredMoment))  # 矩[a, b, a^2, ab, b^2]
plotTime = 0  # 是否画图时间戳
errAll = numpy.zeros(5)
err_x = []
err_y = [[] for j in range(len(desiredMoment))]
position_x = []
position_y = []
i = 0
while i < count:  # 创建count个粒子
    element.append(particle.Particle(i, desiredMoment))
    position_x.append(element[i].positionX)
    position_y.append(element[i].positionY)
    i = i + 1

plt.ion()
fig = plt.figure()
ax = fig.add_subplot(111)
line1, = ax.plot(position_x, position_y, 'ro')  # Returns a tuple of line objects, thus the comma
# fig.show()
fine = 0
while fine == 0:
    '''一次循环'''
    # fine = fine + 1
    plotTime = plotTime + 1
    i = 0
    while i < count:  # 1.程序主体
        element[i].compute_phi()
        element[i].find_neighbour(element)
        element[i].estimation_pi()
        element[i].control_run_pi()
        i = i + 1

    moment = numpy.zeros(len(desiredMoment))  # 2.计算总体moment
    for i in range(count):
        moment = moment + element[i].phi
    moment = moment / count

    errAll = moment - desiredMoment  # 3.计算偏差
    if plotTime % 50 == 0:
        print("moment:", moment)
        '''drawing'''  # 4.画粒子动态反应图
        x = []
        y = []
        for piu in element:
            x.append(piu.positionX)
            y.append(piu.positionY)
        plt.xlim(-10, 10)
        plt.ylim(-10, 10)
        line1.set_xdata(x)
        line1.set_ydata(y)
        fig.canvas.draw()
        # fig.canvas.flush_events()
        err_x.append(plotTime)
        for i in range(len(desiredMoment)):
            err_y[i].append(errAll[i])

    if plotTime > 20000:  # 4.判断是否结束.未结束则继续移动，结束后画出误差图
        print("moment:" + str(moment))
        print("err:" + str(errAll))
        fig2 = plt.figure()
        for i in range(len(desiredMoment)):
            plt.plot(err_x, err_y[i])
        break
