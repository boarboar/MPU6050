import wx
import wx.lib.newevent
import socket
import logging
import model
import controller
import config
import history
import draw
import map

LogEvent, EVT_LOG_EVENT = wx.lib.newevent.NewEvent()
UpdEvent, EVT_UPD_EVENT = wx.lib.newevent.NewEvent()
UpdStatusEvent, EVT_UPD_STAT_EVENT = wx.lib.newevent.NewEvent()

class MyForm(wx.Frame):
    LOG_LINES = 36
    def __init__(self):
        wx.Frame.__init__(self, None, title="Console", size=(800,720))
        self.Bind(wx.EVT_CLOSE, self.OnClose)

        self.model=model.Model("ROBO")
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
        self.btn_sim = wx.Button(panel, wx.ID_ANY, 'Track')
        self.btn_plan = wx.Button(panel, wx.ID_ANY, 'Plan')
        self.btn_go = wx.Button(panel, wx.ID_ANY, 'Go!')
        self.txt_cmd = wx.TextCtrl(panel, style=wx.TE_PROCESS_ENTER, value='{"C":"INFO"}')
        self.btn_cmd = wx.Button(panel, wx.ID_ANY, 'Send')
        bsz=28
        self.btn_map_zoom_in = wx.Button(panel, wx.ID_ANY, '+', size=( bsz,  bsz))
        self.btn_map_zoom_out = wx.Button(panel, wx.ID_ANY, '-', size=( bsz,  bsz))
        self.btn_map_left = wx.Button(panel, wx.ID_ANY, u"\u25C0", size=( bsz,  bsz))
        self.btn_map_right = wx.Button(panel, wx.ID_ANY, u"\u25B6", size=( bsz,  bsz))
        self.btn_map_up = wx.Button(panel, wx.ID_ANY, u"\u25B2", size=( bsz,  bsz))
        self.btn_map_dn = wx.Button(panel, wx.ID_ANY, u"\u25BC", size=( bsz,  bsz))
        self.btn_map_fit = wx.Button(panel, wx.ID_ANY, u"\u25AD", size=( bsz,  bsz))
        self.btn_map_pos = wx.ToggleButton(panel, wx.ID_ANY, u"\u25C9", size=( bsz,  bsz))
        self.btn_map_targ = wx.ToggleButton(panel, wx.ID_ANY, u"\u2605", size=( bsz,  bsz))

        self.btn_mov_left = wx.Button(panel, wx.ID_ANY, u"\u2190", size=( bsz,  bsz))
        self.btn_mov_right = wx.Button(panel, wx.ID_ANY, u"\u2192", size=( bsz,  bsz))
        self.btn_mov_up = wx.Button(panel, wx.ID_ANY, u"\u2191", size=( bsz,  bsz))
        self.btn_mov_dn = wx.Button(panel, wx.ID_ANY, u"\u2193", size=( bsz,  bsz))
        self.btn_mov_stop = wx.Button(panel, wx.ID_ANY, u"\u2717", size=( bsz,  bsz))


        self.unitPan = draw.UnitPanel(panel)
        self.chart = draw.ChartPanel(panel)
        self.map = map.MapPanel(panel, self.model, "map.json", self.LogString, self.LogErrorString)

        self.log_bg=self.log.GetBackgroundColour()

        self.layout(panel)
        # redirect text here
        #sys.stdout=self.log
        #sys.stderr=self.log
        self.logcnt=0
        self.Bind(EVT_LOG_EVENT, self.onLogEvent)
        self.Bind(EVT_UPD_EVENT, self.onUpdEvent)
        self.Bind(EVT_UPD_STAT_EVENT, self.onUpdStatEvent)
        self.Bind(wx.EVT_BUTTON, self.onStatusReq, self.btn_st)
        self.Bind(wx.EVT_BUTTON, self.onPositionReq, self.btn_pos)
        self.Bind(wx.EVT_BUTTON, self.onUploadReq, self.btn_upl)
        self.Bind(wx.EVT_BUTTON, self.onResetMPUReq, self.btn_rst_mpu)
        self.Bind(wx.EVT_BUTTON, self.onResetMPUIntReq, self.btn_rst_int)
        self.Bind(wx.EVT_BUTTON, self.onDumpModel, self.btn_dump)
        self.Bind(wx.EVT_BUTTON, self.onHistory, self.btn_hist)
        self.Bind(wx.EVT_BUTTON, self.onSendCmd, self.btn_cmd)
        self.Bind(wx.EVT_BUTTON, self.onSimReq, self.btn_sim)
        self.Bind(wx.EVT_BUTTON, self.onPlanReq, self.btn_plan)
        self.Bind(wx.EVT_BUTTON, self.onGoReq, self.btn_go)
        self.Bind(wx.EVT_BUTTON, lambda evt, zoom='in': self.map.onZoom(evt, zoom), self.btn_map_zoom_in)
        self.Bind(wx.EVT_BUTTON, lambda evt, zoom='out': self.map.onZoom(evt, zoom), self.btn_map_zoom_out)
        self.Bind(wx.EVT_BUTTON, lambda evt, move='left': self.map.onButtonMove(evt, move), self.btn_map_left)
        self.Bind(wx.EVT_BUTTON, lambda evt, move='right': self.map.onButtonMove(evt, move), self.btn_map_right)
        self.Bind(wx.EVT_BUTTON, lambda evt, move='up': self.map.onButtonMove(evt, move), self.btn_map_up)
        self.Bind(wx.EVT_BUTTON, lambda evt, move='dn': self.map.onButtonMove(evt, move), self.btn_map_dn)
        self.Bind(wx.EVT_BUTTON, self.map.onFit, self.btn_map_fit)
        self.Bind(wx.EVT_TOGGLEBUTTON,
                  lambda evt : self.map.onPosToggle(evt, 1, self.btn_map_pos.GetValue()) and self.btn_map_targ.SetValue(False),
                  self.btn_map_pos)
        self.Bind(wx.EVT_TOGGLEBUTTON,
                  lambda evt : self.map.onPosToggle(evt, 2, self.btn_map_targ.GetValue()) and self.btn_map_pos.SetValue(False),
                  self.btn_map_targ)
        self.txt_cmd.Bind(wx.EVT_KEY_DOWN, self.onEnterCmdText)

        self.Bind(wx.EVT_BUTTON, lambda evt, move=(-0.4, 0.4): self.controller.reqMove(move[0], move[1]), self.btn_mov_left)
        self.Bind(wx.EVT_BUTTON, lambda evt, move=(0.4, -0.4): self.controller.reqMove(move[0], move[1]), self.btn_mov_right)
        self.Bind(wx.EVT_BUTTON, lambda evt, move=(0.4, 0.4): self.controller.reqMove(move[0], move[1]), self.btn_mov_up)
        self.Bind(wx.EVT_BUTTON, lambda evt, move=(-0.4, -0.4): self.controller.reqMove(move[0], move[1]), self.btn_mov_dn)
        self.Bind(wx.EVT_BUTTON, lambda evt, move=(0.0, 0.0): self.controller.reqMove(move[0], move[1]), self.btn_mov_stop)

        self.config=config.Config(self, self.model)
        self.controller=controller.Controller(self, self.model)
        self.LogString("Local address is %s" % socket.gethostbyname(socket.gethostname()))
        self.map.Reset() # init particles filter and position
        self.history = self.model["CMD_HIST"]
        if len(self.history) > 0 :
            self.history_ptr=len(self.history)-1
            self.txt_cmd.SetValue(self.history[self.history_ptr])


    def layout(self, panel):
        self.unitPan.SetMaxSize((240, 240))
        #self.canvas.SetMaxSize((240, 240))
        # Add widgets to a sizer
        sizer = wx.BoxSizer(wx.VERTICAL)
        sizer_pan = wx.BoxSizer(wx.HORIZONTAL)
        sizer_charts = wx.BoxSizer(wx.VERTICAL)
        sizer_ctrls = wx.BoxSizer(wx.VERTICAL)
        sizer_map_ctrls = wx.BoxSizer(wx.HORIZONTAL)
        sizer_map = wx.BoxSizer(wx.VERTICAL)
        sizer_cmd = wx.BoxSizer(wx.HORIZONTAL)

        sizer.Add(sizer_pan, 2, wx.ALL|wx.EXPAND, 5)
        #sizer.Add(self.grid, 0, wx.ALL|wx.EXPAND, 5)
        sizer.Add(self.log, 1, wx.ALL|wx.EXPAND, 5)
        sizer.Add(sizer_cmd, 0, wx.ALL|wx.EXPAND, 5)

        ##sizer_pan.Add(self.unitPan, 1, wx.ALL|wx.EXPAND, 5)
        sizer_charts.Add(self.unitPan, 1, wx.ALL|wx.EXPAND, border=0)
        sizer_charts.Add(self.chart, 1, wx.ALL|wx.EXPAND, border=0)
        sizer_pan.Add(sizer_charts, 1, wx.ALL|wx.EXPAND, border=0)

        #sizer_pan.Add(self.map, 2, wx.ALL|wx.EXPAND, border=0)
        sizer_pan.Add(sizer_map, 2, wx.ALL|wx.EXPAND, border=0)
        sizer_map.Add(self.map, 5, wx.ALL|wx.EXPAND, border=0)
        sizer_map.Add(sizer_map_ctrls, 0, wx.ALL|wx.EXPAND, border=0)
        sizer_map_ctrls.Add(self.btn_map_zoom_in, 0, wx.LEFT|wx.BOTTOM, 0)
        sizer_map_ctrls.Add(self.btn_map_zoom_out, 0, wx.LEFT|wx.BOTTOM, 0)
        sizer_map_ctrls.Add(self.btn_map_left, 0, wx.LEFT|wx.BOTTOM, 0)
        sizer_map_ctrls.Add(self.btn_map_right, 0, wx.LEFT|wx.BOTTOM, 0)
        sizer_map_ctrls.Add(self.btn_map_up, 0, wx.LEFT|wx.BOTTOM, 0)
        sizer_map_ctrls.Add(self.btn_map_dn, 0, wx.LEFT|wx.BOTTOM, 0)
        sizer_map_ctrls.Add(self.btn_map_fit, 0, wx.LEFT|wx.BOTTOM, 0)
        sizer_map_ctrls.Add(self.btn_map_pos, 0, wx.LEFT|wx.BOTTOM, 0)
        sizer_map_ctrls.Add(self.btn_map_targ, 0, wx.LEFT|wx.BOTTOM, 0)

        sizer_map_ctrls.AddSpacer(24)
        sizer_map_ctrls.Add(self.btn_mov_left, 0, wx.LEFT|wx.BOTTOM, 0)
        sizer_map_ctrls.Add(self.btn_mov_right, 0, wx.LEFT|wx.BOTTOM, 0)
        sizer_map_ctrls.Add(self.btn_mov_up, 0, wx.LEFT|wx.BOTTOM, 0)
        sizer_map_ctrls.Add(self.btn_mov_dn, 0, wx.LEFT|wx.BOTTOM, 0)
        sizer_map_ctrls.Add(self.btn_mov_stop, 0, wx.LEFT|wx.BOTTOM, 0)

        sizer_pan.Add(sizer_ctrls, 0, wx.ALL|wx.RIGHT, 5)
        sizer_ctrls.Add(self.btn_st, 0, wx.ALL|wx.CENTER, 5)
        sizer_ctrls.Add(self.btn_pos, 0, wx.ALL|wx.CENTER, 5)
        sizer_ctrls.Add(self.btn_upl, 0, wx.ALL|wx.CENTER, 5)
        sizer_ctrls.Add(self.btn_rst_mpu, 0, wx.ALL|wx.CENTER, 5)
        sizer_ctrls.Add(self.btn_rst_int, 0, wx.ALL|wx.CENTER, 5)
        sizer_ctrls.Add(self.btn_hist, 0, wx.ALL|wx.CENTER, 5)
        sizer_ctrls.Add(self.btn_dump, 0, wx.ALL|wx.CENTER, 5)
        sizer_ctrls.Add(self.btn_sim, 0, wx.ALL|wx.CENTER, 5)
        sizer_ctrls.Add(self.btn_plan, 0, wx.ALL|wx.CENTER, 5)
        sizer_ctrls.Add(self.btn_go, 0, wx.ALL|wx.CENTER, 5)

        sizer_cmd.Add(self.txt_cmd, 10, wx.ALL|wx.CENTER, 5)
        sizer_cmd.Add(self.btn_cmd, 0, wx.ALL|wx.CENTER, 5)

        panel.SetSizer(sizer)
        #panel.SetAutoLayout(True)
        panel.Layout()
        sizer.Fit(panel)


    def AddLine(self, msg, color=None) :
        logging.info(msg)
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

    def UpdateStatus(self, **kwargs) :
        event = UpdStatusEvent(**kwargs)
        wx.PostEvent(self, event)

    def OnClose(self, event):
        print(self.history)
        self.model["CMD_HIST"]=self.history
        self.config.update()
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
        self.controller.reqResetMPU(action="MPU", pos=self.map.start)
        self.map.Reset()
        self.map.UpdateDrawing()

    def onResetMPUIntReq(self, event):
        self.controller.reqResetMPU(action="MPU_INT") # reset integrator
        self.map.Reset()
        self.map.UpdateDrawing()

    def onScanReq(self, event):
        if self.controller.isScanning() :
            self.controller.stopScan()
        else :
            self.controller.startScan()

    def onSimReq(self, event):
        if self.controller.isSimulating() :
            self.controller.stopSimulation()
        else :
            self.controller.startSimulation()

    def onPlanReq(self, event):
        self.map.Plan()

    def onGoReq(self, event):
        if self.controller.isPathRunning() :
            self.controller.stopPathRunning()
        else :
            if len(self.map.planner.spath)<2 :
                self.LogErrorString('No path')
                return
            self.controller.startPathRunning(self.map.planner, self.map.unit)

    def onDumpModel(self, event):
        self.LogString(str(self.model.dump()))

    def onSendCmd(self, event):
        cmd=self.txt_cmd.GetValue()
        self.controller.reqCmdRaw(cmd)
        if len(self.history)==0 or self.history[-1] != cmd :
            if len(self.history) > 10 : self.history.pop(0)
            self.history.append(cmd)
        self.history_ptr=len(self.history)-1

    def onEnterCmdText(self, event):
        keycode = event.GetKeyCode()
        if keycode == wx.WXK_RETURN or keycode == wx.WXK_NUMPAD_ENTER:
            self.onSendCmd(event=None)
            #event.EventObject.Navigate()
        elif keycode == wx.WXK_UP or keycode == wx.WXK_NUMPAD_UP:
            #self.txt_cmd.SetValue('UP')
            if self.history_ptr > 0 :
                self.history_ptr-=1;
                self.txt_cmd.SetValue(self.history[self.history_ptr])
        elif keycode == wx.WXK_DOWN or keycode == wx.WXK_NUMPAD_DOWN:
            #self.txt_cmd.SetValue('DN')
            if self.history_ptr < len(self.history)-1 :
                self.history_ptr+=1;
                self.txt_cmd.SetValue(self.history[self.history_ptr])
        event.Skip()

    def onLogEvent(self, evt):
        self.AddLine(evt.msg, evt.color)

    def onUpdEvent(self, evt):
        #self.statusbar.SetStatusText(str(self.model["FHS"]), 0)
        self.statusbar.SetStatusText("%(YPR)s" % self.model, 1)
        if evt.reset==True :
            self.chart.Reset()
            self.map.Reset()
        else :
            self.map.UpdateData()
            #if self.controller.isPathRunning() :
            #    self.controller.updatePathRunning()

        self.unitPan.UpdateData(self.model["T_ATT"], self.model["YPR"], self.model["V"], self.map.unit.a_mean)
        self.chart.UpdateData(self.model["T_ATT"], self.model["YPR"], self.model["V"])


    def onUpdStatEvent(self, evt):
        self.statusbar.SetStatusText(str(self.model["FHS"]), 0)

class SettingsDialog(wx.Dialog):
    def __init__(self, model, *args, **kw):
        super(SettingsDialog, self).__init__(*args, **kw)
        self.model = model
        self.InitUI()
        self.SetSize((250, 300))
        self.SetTitle("Settings")

    def InitUI(self):

        pnl = wx.Panel(self)
        self.addr = wx.TextCtrl(pnl)
        self.port = wx.TextCtrl(pnl)
        self.listenport = wx.TextCtrl(pnl)
        #self.syslogenable = wx.CheckBox(pnl)
        self.syslogenable = wx.ListBox(pnl, style=wx.LB_SINGLE)
        self.syslogenable.InsertItems(["None", "Alarm", "Message"], 0)
        try:
            self.addr.SetValue(str(self.model["DEVADDR"]))
            self.port.SetValue(str(self.model["DEVPORT"]))
            self.listenport.SetValue(str(self.model["LISTENPORT"]))
            #self.syslogenable.SetValue(int(self.model["SYSLOGENABLE"]))
            self.syslogenable.Select(int(self.model["SYSLOGENABLE"]))
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
        #if self.syslogenable.GetValue() : self.model["SYSLOGENABLE"] = 1
        #else : self.model["SYSLOGENABLE"] = 0

        self.model["SYSLOGENABLE"] = self.syslogenable.GetSelection()

        self.SetReturnCode(True)
        self.Destroy()


# Run the program
if __name__ == "__main__":
    logging.basicConfig(filename='console.log',level=logging.INFO)
    app = wx.App(False)
    frame = MyForm().Show()
    app.MainLoop()
