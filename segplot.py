import time
import numpy as np
import matplotlib
matplotlib.use('TKAgg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import serial
from collections import deque
from threading import Thread

class AnalogPlot:
    def __init__(self, strPort, maxLen):
	self.ser = serial.Serial(strPort, 38400, timeout=0.1,xonxoff=0, interCharTimeout=None)
	self.ax = deque([0.0]*maxLen)
	self.ay = deque([0.0]*maxLen)
	self.az = deque([0.0]*maxLen)
	self.maxLen = maxLen
	self.last_received = ''
	thread = Thread(target=self.receiving)
	thread.daemon = True
	thread.start()

    def receiving(self):
	buffer = ''
	while True:
	    buffer = buffer + self.ser.read(self.ser.inWaiting())
	    if '\n' in buffer:
		lines = buffer.split('\n')
		self.last_received = lines[-2]
		buffer = lines[-1]
	    time.sleep(0.02)

    def addToBuf(self, buf, val):
	buf.popleft()
	buf.append(val)
	#if len(buf) < self.maxLen:
	#    buf.append(val)
	#else:
	#    buf.pop()
	#    buf.appendleft(val)

    def add(self, data):
	assert(len(data) == 3)
	self.addToBuf(self.ax, data[0])
	self.addToBuf(self.ay, data[1])
	self.addToBuf(self.az, data[2])

    def update(self, frameNum, a0, a1, a2):
	try:
	    line = self.last_received
	    line = line.split()
	    if (len(line) == 14):
		print line
		try:
		    data = []
		    data.append(float(line[7]))
		    data.append(float(line[11]))
		    data.append(float(line[13])/255.0*100)
		    self.add(data)
		    a0.set_data(range(self.maxLen), self.ax)
		    a1.set_data(range(self.maxLen), self.ay)
		    a2.set_data(range(self.maxLen), self.az)
		except ValueError:
		    time.sleep(.005)
	except (KeyboardInterrupt, SystemExit):
	    self.close()
	    print ('exiting')
	    sys.exit(0)

	return a0,

    def close(self):
	self.ser.flush()
	self.ser.close()

def main():
    analogPlot = AnalogPlot("/dev/tty.usbserial-FTG5WE3F", 500)
    print('plotting data...')
    fig = plt.figure(1)
    fig.suptitle('Segbot PID Control')
    ax = plt.subplot(211)
    #ax = plt.axes(xlim=(0, 100), ylim=(-20, 200))
    ax.set_xlim(0, 500)
    #ax.set_ylim(0, 180)
    ax.set_ylim(85, 100)
    ax.yaxis.grid()
    a2, = ax.plot([],[], label="Angle")
    a1, = ax.plot([],[], label="Set Point")
    handles, labels = ax.get_legend_handles_labels()
    ax.legend(handles, labels)
    ay = plt.subplot(212)
    ay.set_xlim(0, 500)
    ay.set_ylim(-100, 100)
    ay.yaxis.grid()
    #ax = plt.axes(xlim=(0, 100), ylim=(-255, 255))
    a0, = ay.plot([],[], label="Motor Output")
    handles, labels = ay.get_legend_handles_labels()
    ay.legend(handles, labels)
    anim = animation.FuncAnimation(fig, analogPlot.update, fargs=(a1, a2, a0), interval=20)
    mng = plt.get_current_fig_manager()
    #mng.window.showMaximized()
    mng.resize(*mng.window.maxsize())
    plt.show(block=False)
    analogPlot.close()
    print('exiting.')

if __name__=='__main__':
    main()
