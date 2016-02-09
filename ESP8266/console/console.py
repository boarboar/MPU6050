import sys
import wx
import wx.lib.newevent

import json
import draw
import controller
import model

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
        btn_st = wx.Button(panel, wx.ID_ANY, 'Status')
        self.Bind(wx.EVT_BUTTON, self.onStatusReq, btn_st)

        btn_pos = wx.Button(panel, wx.ID_ANY, 'Pos')
        self.Bind(wx.EVT_BUTTON, self.onPositionReq, btn_pos)

        btn_dump = wx.Button(panel, wx.ID_ANY, 'Dump')
        self.Bind(wx.EVT_BUTTON, self.onDumpModel, btn_dump)

        self.canvas = draw.DrawPanel(panel)

        # Add widgets to a sizer
        sizer = wx.BoxSizer(wx.VERTICAL)
        sizer_pan = wx.BoxSizer(wx.HORIZONTAL)
        sizer_ctrls = wx.BoxSizer(wx.VERTICAL)

        sizer.Add(sizer_pan, 3, wx.ALL|wx.EXPAND, 5)
        sizer.Add(log, 1, wx.ALL|wx.EXPAND, 5)

        sizer_pan.Add(self.canvas, 1, wx.ALL|wx.EXPAND, 5)
        sizer_pan.Add(sizer_ctrls, 0, wx.ALL|wx.RIGHT, 5)
        sizer_ctrls.Add(btn_st, 0, wx.ALL|wx.CENTER, 5)
        sizer_ctrls.Add(btn_pos, 0, wx.ALL|wx.CENTER, 5)
        sizer_ctrls.Add(btn_dump, 0, wx.ALL|wx.CENTER, 5)
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
        self.model=model.Model("ROBO")
        self.controller=controller.Controller(self, self.model)

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

    def UpdatePos(self, **kwargs) :
        event = UpdEvent(**kwargs)
        wx.PostEvent(self, event)

    def onStatusReq(self, event):
        self.controller.reqStatus()

    def onPositionReq(self, event):
        self.controller.reqPosition()

    def onDumpModel(self, event):
        self.LogString(str(self.model))

    def onLogEvent(self, evt):
        self.AddLine(evt.msg)

    def onUpdEvent(self, evt):

        self.AddLine("POS %s as %s" % (self.model["X"], self.model["Y"]))
        self.canvas.AddPoint(self.model["X"], self.model["Y"])

# Run the program
if __name__ == "__main__":
    app = wx.App(False)
    frame = MyForm().Show()
    app.MainLoop()