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
import math
import tf_labels

DNN_PATH = "model/frozen_inference_graph.pb"
DNN_MODEL_PATH = "model/ssd_mobilenet_v1_coco.pbtxt"
DNN_LABELS_PATH = "model/mscoco_label_map.pbtxt"


CameraEvent, EVT_CAMERA_EVENT = wx.lib.newevent.NewEvent()

RedrawEvent, EVT_RDR_EVENT = wx.lib.newevent.NewEvent()

class StreamClientThread(threading.Thread):
    def __init__(self, wnd, url, proxysetting, LogString, LogErrorString, dnn=None):
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
        self.LogString = LogString
        self.LogErrorString = LogErrorString
        self.cvNet=dnn
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

    def obj_detect(self, img):
        rows = img.shape[0]
        cols = img.shape[1]
        self.cvNet.setInput(
            cv2.dnn.blobFromImage(img, 1.0 / 127.5, (300, 300), (127.5, 127.5, 127.5), swapRB=True, crop=False))
        cvOut = self.cvNet.forward()

        for detection in cvOut[0, 0, :, :]:
            score = float(detection[2])
            if score > 0.25:
                left = int(detection[3] * cols)
                top = int(detection[4] * rows)
                right = int(detection[5] * cols)
                bottom = int(detection[6] * rows)
                label = tf_labels.getLabel(int(detection[1]))
                # label = int(detection[1])
                #print(label, score, left, top, right, bottom)
                text_color = (23, 230, 210)
                cv2.rectangle(img, (left, top), (right, bottom), text_color, thickness=1)
                cv2.putText(img, str(label), (left, top), cv2.FONT_HERSHEY_SIMPLEX, 0.5, text_color, 1)

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

                    #return self.edges(img)
                    if self.cvNet is not None:
                        return self.obj_detect(img)
                    else:
                        return img
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
            self.LogString('opening stream at %s ...' % (self.__url))
            self.stream=None
            try:
                self.stream=urllib2.urlopen(self.__url, timeout=10.0)
                print 'stream opened'
                self.LogString('stream opened')
            except URLError as e:
                print e.reason
                continue
            except socket.timeout as e:
                print("timeout")
                self.LogErrorString('stream timeout')
                continue

            self.frame = self.loadimg()

            if self.frame is not None:
                self.height, self.width = self.frame.shape[:2]
                self.bmp = wx.BitmapFromBuffer(self.width, self.height, self.frame)

            else:
                print "Error no webcam image"
                self.LogErrorString('no webcam image')
                continue

            while not self.__stop and self.frame is not None:
                self.lock()
                self.bmp.CopyFromBuffer(self.frame)
                self.unlock()
                #print "Fire event"
                event = RedrawEvent(bmp=self.bmp)
                wx.PostEvent(self.wnd, event)
                time.sleep(0.1)
                self.frame = self.loadimg()

        print "Streamer stopped"


class CameraPanel(wx.Window):
    " camera panel"
    def __init__(self, parent, LogString, LogErrorString):
        wx.Window.__init__(self, parent, wx.ID_ANY, style=wx.SIMPLE_BORDER, size=(160,120))

        #self.isDebug = False
        self.isDebug = True

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
        self.sensors=None
        self.CAM_ANGLE_2_TAN=math.tan(24*math.pi/180) #48 degree view
        self.DIST_THRESH = 50

        self.LogString = LogString
        self.LogErrorString = LogErrorString

        print("Loading DNN...")
        self.LogString('Loading DNN...')
        try:
            tf_labels.initLabels(DNN_LABELS_PATH)
            self.cvNet = cv2.dnn.readNetFromTensorflow(DNN_PATH, DNN_MODEL_PATH)
            self.LogString('DNN loaded')
        except Exception as e:
            print 'failed to read DNN'
            print(e)
            self.LogErrorString('DNN failed to load')
            self.cvNet = None

        print("Done.")

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
        temp_dc = wx.MemoryDC()
        temp_dc.SelectObject(bmp)
        temp_dc.SetBackgroundMode(wx.TRANSPARENT)
        temp_dc.SetBrush(wx.TRANSPARENT_BRUSH)
        temp_dc.SetTextForeground(wx.GREEN)
        temp_dc.SetPen(wx.Pen(wx.GREEN, 1))
        temp_dc.DrawLinePoint(wx.Point(0, Size[1]/2), wx.Point(Size[0]-1, Size[1]/2))
        temp_dc.DrawLinePoint(wx.Point(Size[0]/2, Size[1] / 2-20), wx.Point(Size[0] / 2, Size[1] / 2+20))
        if self.sensors is not None:
            nsens = len(self.sensors)
            isens = nsens/4
            if self.sensors[isens]>0 :
                temp_dc.SetTextForeground(wx.GREEN if self.sensors[isens]>self.DIST_THRESH else wx.RED)
                temp_dc.DrawTextPoint(str(round(self.sensors[isens], 0)), wx.Point(Size[0] / 2, Size[1] / 2 - 20))
            for i in range(1):
                a = 360 / nsens*math.pi/180 *(i+1)
                d = Size[0]*math.tan(a)/(2*self.CAM_ANGLE_2_TAN)
                for j in range(2):
                    k = j*2-1
                    isens = nsens / 4 + k * (i + 1)
                    temp_dc.SetTextForeground(wx.GREEN if self.sensors[isens] > self.DIST_THRESH else wx.RED)
                    if d < Size[0] / 2:
                        temp_dc.DrawLinePoint(wx.Point(Size[0] / 2 + d*k, Size[1] / 2 - 10),
                                      wx.Point(Size[0] / 2 + d*k, Size[1] / 2 + 10))
                        if self.sensors[isens] > 0:
                            temp_dc.DrawTextPoint(str(round(self.sensors[isens], 0)),
                                              wx.Point(Size[0] / 2+d*k, Size[1] / 2-20))
                    else :
                        temp_dc.DrawLinePoint(
                            wx.Point(Size[0] / 2 + (Size[0] / 2 - 10)*k, Size[1] / 2 - 10),
                            wx.Point(Size[0] / 2 + (Size[0] / 2 )*k, Size[1] / 2))
                        temp_dc.DrawLinePoint(
                            wx.Point(Size[0] / 2 + (Size[0] / 2 - 10) * k, Size[1] / 2 + 10),
                            wx.Point(Size[0] / 2 + (Size[0] / 2) * k, Size[1] / 2))

                        if self.sensors[isens] > 0:
                            offset=(k+1)*12
                            temp_dc.DrawTextPoint(str(round(self.sensors[isens], 0)),
                                    wx.Point(Size[0] / 2 + (Size[0] / 2 - offset)*k, Size[1] / 2 - 20))


        temp_dc.SelectObject(wx.NullBitmap)
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
                                                    #"http://iris.not.iac.es/axis-cgi/mjpg/video.cgi?resolution=320x240",
                                                    "http://camera1.mairie-brest.fr/mjpg/video.mjpg?resolution=320x240",
                                                  {'http': 'proxy.reksoft.ru:3128'}, self.LogString, self.LogErrorString, dnn=self.cvNet)
            else :
                self.streamthread =StreamClientThread(self, 'http://192.168.1.120:8080/?action=stream', None, self.LogString, self.LogErrorString, dnn=self.cvNet)
            self.streamthread.start()

    def UpdateData(self, sensors):
        if sensors is not None:
            self.sensors=sensors