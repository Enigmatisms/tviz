"""
    ROS Laser Scan Ternimal Visualizer
"""
import rospy
import curses
import threading
import numpy as np
from drawille import Canvas, animate
from sensor_msgs.msg import LaserScan
from time import sleep

import locale
locale.setlocale(locale.LC_ALL, "")

max_dist = 32.0
min_dist = 0.1

angle_set = False
amin = 0.0
amax = 0.0
ainc = 1.0

cvs = Canvas() 

def animate(canvas, margin, fn, delay=1/24, *args, **kwargs):
    def animation(stdscr):
        for frame in fn(*args, **kwargs):
            stdscr.clear()
            for x,y in frame:
                canvas.set(x,y)
            f = canvas.frame()
            stdscr.addstr(margin, margin, "{0}\n".format(f))
            stdscr.refresh()
            if delay:
                sleep(delay)
            canvas.clear()
    curses.wrapper(animation)

class Drawing(object):
    lock = threading.Lock()
    has_update = False
    pts = []

    def __init__(self, win_w, win_h, margin = 5):
        self.cx, self.cy = win_w // 2, win_h // 2
        self.w = self.cx - margin
        self.h = self.cy - margin
        self.r = self.w / self.h
        self.points = []
        self.angle_lut = np.zeros(0)

    def mainCallBack(self, data):
        try:
            self.callBackDrawing(data)
        except KeyboardInterrupt:
            rospy.core.signal_shutdown('keyboard interrupt')
            exit(0)
        
    def callBackDrawing(self, data):
        global angle_set, amin, ainc
        range_length = len(data.ranges)
        if angle_set == False:
            amin = data.angle_min
            ainc = data.angle_increment
            self.angle_lut = np.array([[np.cos(i * ainc + amin), np.sin(i * ainc + amin)] for i in range(range_length)])
            angle_set = True
        pts_ = np.zeros((range_length, 2))
        for i, r in enumerate(data.ranges):
            if r <= max_dist and r >= min_dist:
                pts_[i, :] = self.angle_lut[i] * r
        max_x = np.max(abs(pts_[:, 0]))
        max_y = np.max(abs(pts_[:, 1]))
        ratio = max_x / max_y
        alpha = self.w / max_x
        if ratio < self.r:
            alpha = self.h / max_y
        pts_ *= alpha
        Drawing.lock.acquire()
        Drawing.pts = [(int(pt[0]), int(pt[1])) for pt in pts_]
        Drawing.has_update = True
        Drawing.lock.release()

    def drawFunction(self):
        while True:
            try:
                Drawing.lock.acquire()
                if Drawing.has_update:
                    self.points = Drawing.pts
                    Drawing.has_update = False
                Drawing.lock.release()
                yield self.points
            except KeyboardInterrupt:
                rospy.core.signal_shutdown('keyboard interrupt')
                exit(0)

class DrawThread(threading.Thread, Drawing):
    def __init__(self, name, canvas, win_w, win_h, margin = 5):
        threading.Thread.__init__(self, name = name)
        Drawing.__init__(self, win_w, win_h, margin)
        self.canvas = canvas
        self.margin = margin

    def run(self):
        animate(self.canvas, self.margin, self.drawFunction, 0.01)

class ListenerThread(threading.Thread, Drawing):
    def __init__(self, name, win_w, win_h, margin = 5):
        threading.Thread.__init__(self, name = name)
        Drawing.__init__(self, win_w, win_h, margin)
        rospy.init_node("tviz", anonymous=True)
        rospy.Subscriber("/scan", LaserScan, self.mainCallBack)

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    cvs = Canvas()
    dt = DrawThread("drawing", cvs, 180, 160, 3)
    lt = ListenerThread("listen", 180, 160, 3)
    dt.start()
    lt.start()
