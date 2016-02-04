import sys
import time
import random
import wx
import wx.lib.newevent
import threading
import Queue
import json

#to_network = Queue.Queue()
LogEvent, EVT_LOG_EVENT = wx.lib.newevent.NewEvent()
UpdEvent, EVT_UPD_EVENT = wx.lib.newevent.NewEvent()

class MyThread (threading.Thread):
    def __init__(self,form):
        self.__form = form
        self.__q = Queue.Queue()
        threading.Thread.__init__(self)
    def put(self, msg):
        self.__q.put_nowait(msg)
    def run (self):
      self.__form.LogString("Thread started")
      while 1:
          while not self.__q.empty():
                message = self.__q.get()
                self.__form.LogString(message)
                js = json.dumps({"x": random.random(), "y": random.random()})
                #self.__form.Update("resp", result='succ', x=random.random(), y=random.random())
                self.__form.Update("resp", result='succ', data=js)
        #time.sleep(1)

class DrawPanel(wx.Window):
    def __init__(self, parent):
        wx.Window.__init__(self, parent, wx.ID_ANY, style=wx.SIMPLE_BORDER)
        self.Bind(wx.EVT_PAINT, self.OnPaint)
        self.points=[(0,0), (50, 50)]

    def OnPaint(self, event=None):
        dc = wx.PaintDC(self)
        dc.Clear()
        dc.SetPen(wx.Pen(wx.BLACK, 4))
        #dc.DrawLine(0, 0, 50, 50)
        #if len(self.points)>1 :
        #dc.DrawLine(self.points[0][0], self.points[0][1], self.points[1][0], self.points[1][0])
        for point in self.points:
            dc.DrawCheckMark(point[0], point[1], 10, 10)

    def AddPoint(self, xf, yf):
        x=int(xf*100)
        y=int(yf*100)
        self.points.append((x, y))

        #print self.points

        dc = wx.ClientDC(self)
        dc.DrawCheckMark(x, y, 10, 10)
        dc.SetPen(wx.Pen(wx.BLACK, 4))


class MyForm(wx.Frame):
    LOG_LINES = 5
    def __init__(self):
        wx.Frame.__init__(self, None,
                          title="Console")

        # Add a panel so it looks the correct on all platforms
        panel = wx.Panel(self, wx.ID_ANY)
        style = wx.TE_MULTILINE|wx.TE_READONLY|wx.HSCROLL
        log = wx.TextCtrl(panel, wx.ID_ANY, size=(300,100),
                          style=style)
        btn = wx.Button(panel, wx.ID_ANY, 'Send')
        self.Bind(wx.EVT_BUTTON, self.onButton, btn)

        btn1 = wx.Button(panel, wx.ID_ANY, 'Log test')
        self.Bind(wx.EVT_BUTTON, self.onButton1, btn1)

        self.canvas = DrawPanel(panel)

        # Add widgets to a sizer
        sizer = wx.BoxSizer(wx.VERTICAL)
        sizer_pan = wx.BoxSizer(wx.HORIZONTAL)
        sizer_ctrls = wx.BoxSizer(wx.VERTICAL)

        sizer.Add(sizer_pan, 3, wx.ALL|wx.EXPAND, 5)
        sizer.Add(log, 1, wx.ALL|wx.EXPAND, 5)

        sizer_pan.Add(self.canvas, 1, wx.ALL|wx.EXPAND, 5)
        sizer_pan.Add(sizer_ctrls, 0, wx.ALL|wx.RIGHT, 5)
        sizer_ctrls.Add(btn, 0, wx.ALL|wx.CENTER, 5)
        sizer_ctrls.Add(btn1, 0, wx.ALL|wx.CENTER, 5)
        panel.SetSizer(sizer)
        panel.SetAutoLayout(True)
        sizer.Fit(panel)
        # redirect text here
        sys.stdout=log
        sys.stderr=log
        self.log=log
        self.logcnt=0
        self.Bind(EVT_LOG_EVENT, self.onLogEvent)
        self.Bind(EVT_UPD_EVENT, self.onUpdEvent)

        #t = threading.Thread(target=loop)
        t=MyThread(self)
        t.setDaemon(1)
        self.worker=t
        t.start()

    def AddLine(self, msg) :
        while self.log.GetNumberOfLines()>self.LOG_LINES:
            self.log.Remove(0, self.log.GetLineLength(0)+1)
        self.log.AppendText("%d : %s" % (self.logcnt, msg))
        if not msg.endswith('\n'):
            self.log.AppendText('\n')
        self.logcnt=self.logcnt+1

    def LogString(self, message, **kwargs):
        event = LogEvent(msg=message, **kwargs)
        wx.PostEvent(self, event)

    def Update(self, message, **kwargs) :
        event = UpdEvent(msg=message, **kwargs)
        wx.PostEvent(self, event)

    def onButton(self, event):
        self.worker.put("Message to send")

    def onButton1(self, event):
        self.LogString("LogTest")

    def onLogEvent(self, evt):
        self.AddLine(evt.msg)

    def onUpdEvent(self, evt):
        self.AddLine("%s as %s with %s" % (evt.msg, evt.result, evt.data))
        parsed_json = json.loads(evt.data)
        self.canvas.AddPoint(parsed_json["x"], parsed_json["y"])

# Run the program
if __name__ == "__main__":
    app = wx.App(False)
    frame = MyForm().Show()
    app.MainLoop()