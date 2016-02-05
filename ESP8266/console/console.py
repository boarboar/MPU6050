import sys
import wx
import wx.lib.newevent

import json
import draw
import comm

LogEvent, EVT_LOG_EVENT = wx.lib.newevent.NewEvent()
UpdEvent, EVT_UPD_EVENT = wx.lib.newevent.NewEvent()

class MyForm(wx.Frame):
    LOG_LINES = 20
    def __init__(self):
        wx.Frame.__init__(self, None,
                          title="Console")

        # Add a panel so it looks the correct on all platforms
        panel = wx.Panel(self, wx.ID_ANY)
        style = wx.TE_MULTILINE|wx.TE_READONLY|wx.HSCROLL
        log = wx.TextCtrl(panel, wx.ID_ANY, size=(400,200),
                          style=style)
        btn = wx.Button(panel, wx.ID_ANY, 'Send')
        self.Bind(wx.EVT_BUTTON, self.onButton, btn)

        btn1 = wx.Button(panel, wx.ID_ANY, 'Log test')
        self.Bind(wx.EVT_BUTTON, self.onButton1, btn1)

        self.canvas = draw.DrawPanel(panel)

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

        self.worker=comm.CommandThread(self)
        self.worker.start()

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