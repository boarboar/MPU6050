import wx
#import sys
import wx.lib.newevent
import cv2
import urllib2
from urllib2 import URLError
import numpy as np
import threading
import time
import socket

CameraEvent, EVT_CAMERA_EVENT = wx.lib.newevent.NewEvent()

RedrawEvent, EVT_RDR_EVENT = wx.lib.newevent.NewEvent()

class StreamClientThread(threading.Thread):
    def __init__(self, wnd, url, proxysetting):
        threading.Thread.__init__(self)
        self.__lock=threading.Lock()
        self.wnd=wnd
        self.__url = url
        self.__proxysetting=proxysetting
        self.__stop = False
        self.stream=None
        self.bytes=''
        self.sigma = 0.33
        self.lines_rho = 1
        self.lines_phi_div = 180
        self.lines_threshold = 100
        self.lines_minLineLength = 56
        self.lines_maxLineGap = 100
        self.setDaemon(1)
    def stop(self) : self.__stop=True
    def lock(self) : self.__lock.acquire()
    def unlock(self) : self.__lock.release()


    def edges(self, img):
        h, w = img.shape[:2]
        low_bound = 0.05
        hi_bound = 0.5
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # auto canny
        v = np.median(gray)
        # apply automatic Canny edge detection using the computed median
        low_thresh = int(max(0, (1.0 - self.sigma) * v))
        high_thresh = int(min(255, (1.0 + self.sigma) * v))
        edges = cv2.Canny(gray, low_thresh, high_thresh, apertureSize=3, L2gradient=False)
        lines = cv2.HoughLinesP(edges, self.lines_rho, np.pi / self.lines_phi_div,
                                threshold=self.lines_threshold, minLineLength=self.lines_minLineLength,
                                maxLineGap=self.lines_maxLineGap)
        if lines is None:
            return img
<<<<<<< HEAD

=======
>>>>>>> 12068d653c5743b9798bda770397e59c58f71c10

        print("Lines : %s" % (len(lines)))

        y_lim_1 = int(h * (1-low_bound))
        y_lim_0 = int(h * (1 - hi_bound))
        #cv2.line(img, (0, y_lim_1), (w-1, y_lim_1), (0, 0, 255), 1)
        #cv2.line(img, (0, y_lim_0), (w - 1, y_lim_0), (0, 0, 255), 1)

        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(img, (x1, y1), (x2, y2), (0, 255, 0), 3)

        #for line in lines:
        #    for x1, y1, x2, y2 in line:
        #        if abs(y1-y2)>40 and abs(x1-x2)*100/abs(y1-y2)<10 and min(y1,y2)<y_lim_1 and max(y1,y2)>y_lim_0:  #5% inclination
        #            cv2.line(img, (x1, y1), (x2, y2), (255, 0, 0), 2)

        return img

    def loadimg(self):
        if self.stream is None : return None
        while True:
            try :
                self.bytes+=self.stream.read(1024)
                a = self.bytes.find('\xff\xd8')
                b = self.bytes.find('\xff\xd9')
                if a!=-1 and b!=-1:
                    jpg = self.bytes[a:b+2]
                    self.bytes= self.bytes[b+2:]
                    img = cv2.imdecode(np.fromstring(jpg, dtype=np.uint8),cv2.IMREAD_COLOR)
                    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
<<<<<<< HEAD
                    #gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                    return self.edges(img)
=======
                    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                    return self.edges(img, gray)
>>>>>>> 12068d653c5743b9798bda770397e59c58f71c10
                    #return img
            except Exception as e:
                print 'failed to read'
                print(e)
                return None

    def run (self):

        print 'starting srteamer...'

        if self.__proxysetting is not None :
            proxy = urllib2.ProxyHandler(self.__proxysetting)
            opener = urllib2.build_opener(proxy)
            urllib2.install_opener(opener)

        while not self.__stop:
            time.sleep(5.0)
            print('opening stream at %s ...' % (self.__url))
            self.stream=None
            try:
                self.stream=urllib2.urlopen(self.__url, timeout=10.0)
                print 'stream opened'
            except URLError as e:
                print e.reason
                continue
            except socket.timeout as e:
                print("timeout")
                continue

            self.frame = self.loadimg()

            if self.frame is not None:
                self.height, self.width = self.frame.shape[:2]
                self.bmp = wx.BitmapFromBuffer(self.width, self.height, self.frame)

            else:
                print "Error no webcam image"
                continue

            while not self.__stop and self.frame is not None:
                self.lock()
                self.bmp.CopyFromBuffer(self.frame)
                self.unlock()
                #print "Fire event"
                event = RedrawEvent(bmp=self.bmp)
                wx.PostEvent(self.wnd, event)
                time.sleep(0.05)
                self.frame = self.loadimg()

        print "Streamer stopped"


class CameraPanel(wx.Window):
    " camera panel"
    def __init__(self, parent):
        wx.Window.__init__(self, parent, wx.ID_ANY, style=wx.SIMPLE_BORDER, size=(160,120))

        self.isDebug = False

        #self.imgSizer = (480, 360)
        self.imgSizer = (640, 480)
        self.pnl = wx.Panel(self, -1)
        sizer = wx.BoxSizer(wx.VERTICAL)
        sizer.Add(self.pnl, 1, flag=wx.EXPAND)
        self.SetSizer(sizer)

        #self.vbox = wx.BoxSizer(wx.VERTICAL)

        self.image = wx.EmptyImage(self.imgSizer[0],self.imgSizer[1])
        self.imageBit = wx.BitmapFromImage(self.image)
        self.staticBit = wx.StaticBitmap(self.pnl, wx.ID_ANY, self.imageBit)

        #self.vbox.Add(self.staticBit)

        self.Bind(wx.EVT_PAINT, self.OnPaint)
        self.Bind(EVT_RDR_EVENT, self.onRedrawEvent)

        self.isPlaying = False
        self.staticBit.Bind(wx.EVT_LEFT_UP, self.OnMouseLeftUp)

        #self.SetSize(self.imgSizer)
        #self.pnl.SetSizer(self.vbox)
        #self.vbox.Fit(self)
        #self.Show()

    def onRedrawEvent(self, evt):
        #print "Update"
        """
        self.streamthread.lock()
        self.staticBit.SetBitmap(evt.bmp)
        self.Refresh()
        self.streamthread.unlock()
        """
        #Size  = self.staticBit.ClientSize
        Size  = self.pnl.ClientSize
        self.streamthread.lock()
        image=wx.ImageFromBitmap(evt.bmp)
        self.streamthread.unlock()
        image = image.Scale(Size[0], Size[1], wx.IMAGE_QUALITY_HIGH)
        bmp = wx.BitmapFromImage(image)
        self.staticBit.SetBitmap(bmp)
        self.Refresh()


    def OnPaint(self, event):
        """
        if self.isPlaying :
            self.streamthread.lock()
            self.Refresh()
            self.streamthread.unlock()
        """
        self.Refresh()

    def OnMouseLeftUp(self, evt):
        if self.isPlaying :
            self.isPlaying=False
            self.streamthread.stop()
            self.streamthread = None
        else :
            self.isPlaying=True
            if self.isDebug :
                self.streamthread =StreamClientThread(self,
                                                  #"http://88.53.197.250/axis-cgi/mjpg/video.cgi?resolution=320x240",
                                                    "http://iris.not.iac.es/axis-cgi/mjpg/video.cgi?resolution=320x240",
                                                  {'http': 'proxy.reksoft.ru:3128'})
            else :
                self.streamthread =StreamClientThread(self, 'http://192.168.1.120:8080/?action=stream', None)
            self.streamthread.start()
