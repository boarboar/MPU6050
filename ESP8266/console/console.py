import wx
import wx.lib.newevent
import socket
import draw
import model
import controller
import config
import history

LogEvent, EVT_LOG_EVENT = wx.lib.newevent.NewEvent()
UpdEvent, EVT_UPD_EVENT = wx.lib.newevent.NewEvent()

class MyForm(wx.Frame):
    LOG_LINES = 20
    def __init__(self):
        wx.Frame.__init__(self, None, title="Console", size=(640,480))
        self.Bind(wx.EVT_CLOSE, self.OnClose)
        menuBar = wx.MenuBar()
        menu = wx.Menu()
        menuBar.Append(menu, "&File")
        m_setup = menu.Append(wx.ID_SETUP, "&Setup")
        m_scan = menu.Append(wx.ID_FILE1, "&Scan Start/Stop")
        self.Bind(wx.EVT_MENU, self.OnSetup, m_setup)
        self.Bind(wx.EVT_MENU, self.onScanReq, m_scan)
        self.SetMenuBar(menuBar)
        self.statusbar = self.CreateStatusBar()
        self.statusbar.SetFieldsCount(3)
        self.statusbar.SetStatusText("---", 0)
        self.statusbar.SetStatusText("---,----", 1)
        # Add a panel so it looks the correct on all platforms
        panel = wx.Panel(self, wx.ID_ANY)
        self.log = wx.TextCtrl(panel, wx.ID_ANY, size=(400,200), style=wx.TE_MULTILINE|wx.TE_READONLY|wx.HSCROLL|wx.TE_RICH)
        self.btn_st = wx.Button(panel, wx.ID_ANY, 'Status')
        self.btn_pos = wx.Button(panel, wx.ID_ANY, 'Pos')
        self.btn_upl = wx.Button(panel, wx.ID_ANY, 'Upload')
        self.btn_rst_mpu = wx.Button(panel, wx.ID_ANY, 'ResetMPU')
        self.btn_rst_int = wx.Button(panel, wx.ID_ANY, 'ResetINT')
        self.btn_hist = wx.Button(panel, wx.ID_ANY, 'Measmts')
        self.btn_dump = wx.Button(panel, wx.ID_ANY, 'Dump')
        self.txt_cmd = wx.TextCtrl(panel)
        self.btn_cmd = wx.Button(panel, wx.ID_ANY, 'Send')

        self.unitPan = draw.UnitPanel(panel)
        self.chart = draw.ChartPanel(panel)
        self.map = draw.MapPanel(panel)

        self.unitPan.SetMaxSize((240, 240))
        #self.canvas.SetMaxSize((240, 240))

        self.log_bg=self.log.GetBackgroundColour()

        self.layout(panel)
        # redirect text here
        #sys.stdout=self.log
        #sys.stderr=self.log
        self.logcnt=0
        self.Bind(EVT_LOG_EVENT, self.onLogEvent)
        self.Bind(EVT_UPD_EVENT, self.onUpdEvent)
        self.Bind(wx.EVT_BUTTON, self.onStatusReq, self.btn_st)
        self.Bind(wx.EVT_BUTTON, self.onPositionReq, self.btn_pos)
        self.Bind(wx.EVT_BUTTON, self.onUploadReq, self.btn_upl)
        self.Bind(wx.EVT_BUTTON, self.onResetMPUReq, self.btn_rst_mpu)
        self.Bind(wx.EVT_BUTTON, self.onResetMPUIntReq, self.btn_rst_int)
        self.Bind(wx.EVT_BUTTON, self.onDumpModel, self.btn_dump)
        self.Bind(wx.EVT_BUTTON, self.onHistory, self.btn_hist)
        self.Bind(wx.EVT_BUTTON, self.onSendCmd, self.btn_cmd)
        self.model=model.Model("ROBO")
        self.config=config.Config(self, self.model)
        self.controller=controller.Controller(self, self.model)
        self.LogString("Local address is %s" % socket.gethostbyname(socket.gethostname()))

    def layout(self, panel):
        # Add widgets to a sizer
        sizer = wx.BoxSizer(wx.VERTICAL)
        sizer_pan = wx.BoxSizer(wx.HORIZONTAL)
        sizer_charts = wx.BoxSizer(wx.VERTICAL)
        sizer_ctrls = wx.BoxSizer(wx.VERTICAL)
        sizer_cmd = wx.BoxSizer(wx.HORIZONTAL)

        sizer.Add(sizer_pan, 2, wx.ALL|wx.EXPAND, 5)
        #sizer.Add(self.grid, 0, wx.ALL|wx.EXPAND, 5)
        sizer.Add(self.log, 1, wx.ALL|wx.EXPAND, 5)
        sizer.Add(sizer_cmd, 0, wx.ALL|wx.EXPAND, 5)

        ##sizer_pan.Add(self.unitPan, 1, wx.ALL|wx.EXPAND, 5)
        sizer_charts.Add(self.unitPan, 1, wx.ALL|wx.EXPAND, border=0)
        sizer_charts.Add(self.chart, 1, wx.ALL|wx.EXPAND, border=0)
        sizer_pan.Add(sizer_charts, 1, wx.ALL|wx.EXPAND, border=0)

        sizer_pan.Add(self.map, 1, wx.ALL|wx.EXPAND, border=0)
        sizer_pan.Add(sizer_ctrls, 0, wx.ALL|wx.RIGHT, 5)
        sizer_ctrls.Add(self.btn_st, 0, wx.ALL|wx.CENTER, 5)
        sizer_ctrls.Add(self.btn_pos, 0, wx.ALL|wx.CENTER, 5)
        sizer_ctrls.Add(self.btn_upl, 0, wx.ALL|wx.CENTER, 5)
        sizer_ctrls.Add(self.btn_rst_mpu, 0, wx.ALL|wx.CENTER, 5)
        sizer_ctrls.Add(self.btn_rst_int, 0, wx.ALL|wx.CENTER, 5)
        sizer_ctrls.Add(self.btn_hist, 0, wx.ALL|wx.CENTER, 5)
        sizer_ctrls.Add(self.btn_dump, 0, wx.ALL|wx.CENTER, 5)

        sizer_cmd.Add(self.txt_cmd, 10, wx.ALL|wx.CENTER, 5)
        sizer_cmd.Add(self.btn_cmd, 0, wx.ALL|wx.CENTER, 5)

        panel.SetSizer(sizer)
        #panel.SetAutoLayout(True)
        panel.Layout()
        sizer.Fit(panel)


    def AddLine(self, msg, color) :
        while self.log.GetNumberOfLines()>self.LOG_LINES:
            self.log.Remove(0, self.log.GetLineLength(0)+1)
        if color is None : color= wx.BLACK

        self.log.SetDefaultStyle(wx.TextAttr(color,self.log_bg))

        # self.log.AppendText("%d : %s" % (self.logcnt, msg))
        self.log.AppendText(msg)
        if not msg.endswith('\n'):
            self.log.AppendText('\n')
        self.logcnt=self.logcnt+1

    def LogString(self, message, color='BLACK') :
        event = LogEvent(msg=message, color=color)
        wx.PostEvent(self, event)

    def LogErrorString(self, message) :
        self.LogString(message, color='RED')

    def UpdatePos(self, **kwargs) :
        event = UpdEvent(**kwargs)
        wx.PostEvent(self, event)

    def OnClose(self, event):
        self.controller.stop(timeout=10.0)
        self.Destroy()

    def OnSetup(self, event):
        sd = SettingsDialog(self.model, None)
        sd.ShowModal()
        rc = sd.GetReturnCode()
        sd.Destroy()
        if rc :
            self.config.update()
            self.controller.restart()

    def onHistory(self, event):
        sd = history.HistoryDialog(self, self.model, None)
        sd.ShowModal()
        sd.Destroy()

    def onStatusReq(self, event):
        self.controller.reqStatus()

    def onPositionReq(self, event):
        self.controller.reqPosition()

    def onUploadReq(self, event):
        self.controller.reqUpload()

    def onResetMPUReq(self, event):
        self.controller.reqResetMPU(action="MPU")

    def onResetMPUIntReq(self, event):
        self.controller.reqResetMPU(action="MPU_INT") # reset integrator

    def onScanReq(self, event):
        if self.controller.isScanning() :
            self.controller.stopScan()
        else :
            self.controller.startScan()

    def onDumpModel(self, event):
        self.LogString(str(self.model.dump()))

    def onSendCmd(self, event):
        self.controller.reqCmdRaw(self.txt_cmd.GetValue())

    def onLogEvent(self, evt):
        self.AddLine(evt.msg, evt.color)

    def onUpdEvent(self, evt):
        self.statusbar.SetStatusText(str(self.model["FHS"]), 0)
        self.statusbar.SetStatusText("%(YPR)s" % self.model, 1)
        if evt.reset==True :
            self.chart.Reset()
        self.unitPan.UpdateData(self.model["T_ATT"], self.model["YPR"], self.model["V"])
        self.chart.UpdateData(self.model["T_ATT"], self.model["YPR"], self.model["V"])
        self.map.UpdateData(self.model["T_ATT"], None)

class SettingsDialog(wx.Dialog):
    def __init__(self, model, *args, **kw):
        super(SettingsDialog, self).__init__(*args, **kw)
        self.model = model
        self.InitUI()
        self.SetSize((250, 200))
        self.SetTitle("Settings")

    def InitUI(self):

        pnl = wx.Panel(self)
        self.addr = wx.TextCtrl(pnl)
        self.port = wx.TextCtrl(pnl)
        self.listenport = wx.TextCtrl(pnl)
        self.syslogenable = wx.CheckBox(pnl)

        try:
            self.addr.SetValue(str(self.model["DEVADDR"]))
            self.port.SetValue(str(self.model["DEVPORT"]))
            self.listenport.SetValue(str(self.model["LISTENPORT"]))
            self.syslogenable.SetValue(int(self.model["SYSLOGENABLE"]))
        except KeyError : pass

        vbox = wx.BoxSizer(wx.VERTICAL)

        sb = wx.StaticBox(pnl, label='Device communication')
        sbs = wx.StaticBoxSizer(sb, orient=wx.VERTICAL)

        hbox = wx.BoxSizer(wx.HORIZONTAL)
        hbox.Add(wx.StaticText(pnl, label='IP Addr:'), flag=wx.LEFT, border=15)
        hbox.Add(self.addr, flag=wx.RIGHT, border=15)
        sbs.Add(hbox)
        hbox = wx.BoxSizer(wx.HORIZONTAL)
        hbox.Add(wx.StaticText(pnl, label='IP Port:'), flag=wx.LEFT, border=15)
        hbox.Add(self.port, flag=wx.RIGHT, border=15)
        sbs.Add(hbox)
        hbox = wx.BoxSizer(wx.HORIZONTAL)
        hbox.Add(wx.StaticText(pnl, label='Enable syslog:'), flag=wx.LEFT, border=15)
        hbox.Add(self.syslogenable, flag=wx.RIGHT, border=15)
        sbs.Add(hbox)
        hbox = wx.BoxSizer(wx.HORIZONTAL)
        hbox.Add(wx.StaticText(pnl, label='Local IP Port:'), flag=wx.LEFT, border=15)
        hbox.Add(self.listenport, flag=wx.RIGHT, border=15)
        sbs.Add(hbox)

        pnl.SetSizer(sbs)

        hbox3 = wx.BoxSizer(wx.HORIZONTAL)
        okButton = wx.Button(self, label='Ok')
        closeButton = wx.Button(self, label='Close')
        hbox3.Add(okButton)
        hbox3.Add(closeButton, flag=wx.LEFT, border=5)

        vbox.Add(pnl, proportion=1,
            flag=wx.ALL|wx.EXPAND, border=5)
        vbox.Add(hbox3,
            flag=wx.ALIGN_CENTER|wx.TOP|wx.BOTTOM, border=10)

        self.SetSizer(vbox)
        vbox.Fit(self)

        okButton.Bind(wx.EVT_BUTTON, self.OnOk)
        closeButton.Bind(wx.EVT_BUTTON, self.OnClose)

    def OnClose(self, e):
        self.SetReturnCode(False)
        self.Destroy()

    def OnOk(self, e):
        try:
            port = int(self.port.GetValue())
        except ValueError : port=-1
        if port<=0 or port>64000 :
            self.port.SetFocus()
            return
        try:
            listenport = int(self.listenport.GetValue())
        except ValueError : listenport=-1
        if listenport<=0 or listenport>64000 :
            self.listenport.SetFocus()
            return
        addr = str(self.addr.GetValue())
        try:
            hn = socket.gethostbyname(addr)
        except socket.gaierror: hn = ""

        if hn=="" :
            self.addr.SetFocus()
            return

        self.model["DEVADDR"] = addr
        self.model["DEVPORT"] = port
        self.model["LISTENPORT"] = listenport
        if self.syslogenable.GetValue() : self.model["SYSLOGENABLE"] = 1
        else : self.model["SYSLOGENABLE"] = 0

        self.SetReturnCode(True)
        self.Destroy()


# Run the program
if __name__ == "__main__":
    app = wx.App(False)
    frame = MyForm().Show()
    app.MainLoop()
