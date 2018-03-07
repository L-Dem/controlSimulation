import matplotlib.pyplot as plt
# import numpy as np

# x = np.linspace(0, 6*np.pi, 100)
# y = np.sin(x)


# You probably won't need this if you're embedding things in a tkinter plot...
plt.ion()

fig = plt.figure()

ax = fig.add_subplot(111)
line1, = ax.plot(1, 1, 'ro')  # Returns a tuple of line objects, thus the comma
# for phase in np.linspace(0, 10*np.pi, 500):
#     line1.set_ydata(np.sin(x + phase))
#     # fig.show()
#     fig.canvas.draw()

fine = 1
position_x = 1
while fine < 100:

    plt.xlim(0, 200)
    plt.ylim(0, 200)
    positionX = []
    positionY = []
    i = 1
    while i < 10:
        positionX.append(position_x + i)
        positionY.append(position_x + i + 1)
        i = i + 1
    line1.set_xdata(positionX)
    line1.set_ydata(positionY)
    position_x = position_x + 1
    print(position_x)
    fine = fine + 1
    # fig.show()
    fig.canvas.draw()
