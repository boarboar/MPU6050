import wx
import wx.lib.newevent
import cv2
import urllib2
from urllib2 import URLError
import numpy as np
import threading
import time

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
        self.setDaemon(1)
    def stop(self) : self.__stop=True
    def lock(self) : self.__lock.acquire()
    def unlock(self) : self.__lock.release()

    def loadimg(self):
        if self.stream is None : return None
        while True:
            try:
                self.bytes+=str(self.stream.read(1024))
                a = self.bytes.find('\xff\xd8')
                b = self.bytes.find('\xff\xd9')
                if a!=-1 and b!=-1:
                    jpg = self.bytes[a:b+2]
                    self.bytes= self.bytes[b+2:]
                    i = cv2.imdecode(np.fromstring(jpg, dtype=np.uint8),cv2.IMREAD_COLOR)
                    i = cv2.cvtColor(i, cv2.COLOR_BGR2RGB)
                    #i = cv2.imdecode(np.fromstring(jpg, dtype=np.uint8),cv2.COLOR_BGR2RGB)
                    return i
                    
            except Exception as e:
                print('failed to read')
                return None
                
    def run (self):
        if self.__proxysetting is not None :
            proxy = urllib2.ProxyHandler(self.__proxysetting)
            opener = urllib2.build_opener(proxy)
            urllib2.install_opener(opener)

        while not self.__stop:
            time.sleep(5.0)
            print('opening stream...')
            self.stream=None
            try:
                self.stream=urllib2.urlopen(self.__url, timeout=10.0)
                print('stream opened')
            except URLError as e:
                print(e.reason)
                continue

            self.frame = self.loadimg()

            if self.frame is not None:
                self.height, self.width = self.frame.shape[:2]
                self.bmp = wx.BitmapFromBuffer(self.width, self.height, self.frame)

            else:
                print("Error no webcam image")
                continue

            while not self.__stop and self.frame is not None:
                #self.lock()
                self.bmp.CopyFromBuffer(self.frame)
                #self.unlock()
                #print("Fire event")
                #event = RedrawEvent(bmp=self.bmp)
                #wx.PostEvent(self.wnd, event)
                self.wnd.staticBit.SetBitmap(self.bmp)
                self.frame = self.loadimg()

class viewWindow(wx.Frame):
    def __init__(self, parent, title="View Window"):
            # super(viewWindow,self).__init__(parent)
            wx.Frame.__init__(self, parent)

            self.imgSizer = (480, 360)
            self.pnl = wx.Panel(self)
            self.vbox = wx.BoxSizer(wx.VERTICAL)

            self.image = wx.EmptyImage(self.imgSizer[0],self.imgSizer[1])
            self.imageBit = wx.BitmapFromImage(self.image)
            self.staticBit = wx.StaticBitmap(self.pnl, wx.ID_ANY, self.imageBit)

            self.vbox.Add(self.staticBit)

            #self.Bind(wx.EVT_PAINT, self.OnPaint)
            #self.Bind(EVT_RDR_EVENT, self.onRedrawEvent)
            self.Bind(wx.EVT_ERASE_BACKGROUND, self.OnEraseBackground)
            self.pnl.Bind(wx.EVT_ERASE_BACKGROUND, self.OnEraseBackground)

            #self.streamthread =StreamClientThread(self,
            #                                      "http://88.53.197.250/axis-cgi/mjpg/video.cgi?resolution=320x240",
            #                                      {'http': 'proxy.reksoft.ru:3128'})

            self.streamthread =StreamClientThread(self, "http://192.168.1.134", None)


            self.streamthread.start()

            self.SetSize(self.imgSizer)
            self.pnl.SetSizer(self.vbox)
            self.vbox.Fit(self)
            self.Show()

    def onRedrawEvent(self, evt):
        #print("Rcv event")
        #self.streamthread.lock()
        #self.staticBit.SetBitmap(evt.bmp)
        #self.Refresh()
        #self.streamthread.unlock()
        pass

    def OnPaint(self, event):
        #self.streamthread.lock()
        #self.Refresh()
        #self.streamthread.unlock()
        pass
        
    def OnEraseBackground(self, event):
        pass



def main():
    app = wx.PySimpleApp()
    frame = viewWindow(None)
    frame.Center()
    frame.Show()
    app.MainLoop()

if __name__ == '__main__':
    main()