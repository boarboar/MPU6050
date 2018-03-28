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
import camera

LogEvent, EVT_LOG_EVENT = wx.lib.newevent.NewEvent()
UpdEvent, EVT_UPD_EVENT = wx.lib.newevent.NewEvent()
UpdStatusEvent, EVT_UPD_STAT_EVENT = wx.lib.newevent.NewEvent()
InfoEvent, EVT_INFO_EVENT = wx.lib.newevent.NewEvent()
ActEvent, EVT_ACT_EVENT = wx.lib.newevent.NewEvent()

class MyForm(wx.Frame):
    def __init__(self):
        wx.Frame.__init__(self, None, title="Console", size=(800,720))
        self.Bind(wx.EVT_CLOSE, self.OnClose)

        self.model=model.Model("ROBO")
        self.config=config.Config(self, self.model)
        self.map_ctrls = []
        self.pan_ctrls = []

        menuBar = wx.MenuBar()
        menu = wx.Menu()
        menuBar.Append(menu, "&File")
        m_setup = menu.Append(wx.ID_SETUP, "&Setup")
        m_scan = menu.Append(wx.ID_FILE1, "S&can Start/Stop")
        m_dump = menu.Append(wx.ID_FILE1, "&Dump")
        self.Bind(wx.EVT_MENU, self.OnSetup, m_setup)
        self.Bind(wx.EVT_MENU, self.onScanReq, m_scan)
        self.Bind(wx.EVT_MENU, self.onDumpModel, m_scan)
        self.SetMenuBar(menuBar)
        self.statusbar = self.CreateStatusBar()
        self.statusbar.SetFieldsCount(4)
        self.statusbar.SetStatusText("---", 0)
        self.statusbar.SetStatusText("---,----", 1)
        self.statusbar.SetStatusText("none", 2)
        self.statusbar.SetStatusText("(0,0)", 3)
        # Add a panel so it looks the correct on all platforms
        panel = wx.Panel(self, wx.ID_ANY)
        self.log = LogControl(panel, sz=(600,200))
        self.log_a = LogControl(panel, sz=(400,200))

        self.btn_st = self.AddPanBtn(panel, 'Status', self.onStatusReq)
        self.btn_pos = self.AddPanBtn(panel, 'Pos', self.onPositionReq)
        self.btn_upl = self.AddPanBtn(panel, 'Upload', self.onUploadReq)
        self.btn_rst_mpu = self.AddPanBtn(panel, 'ResetIMU', self.onResetMPUReq)
        self.btn_rst_int = self.AddPanBtn(panel, 'ResetCTRL', self.onResetMPUIntReq)
        #self.btn_hist = self.AddPanBtn(panel, 'Measmts', self.onHistory)
        #self.btn_dump = self.AddPanBtn(panel, 'Dump', self.onDumpModel)
        self.pan_ctrls.append(None)
        self.btn_track = self.AddPanBtn(panel, 'Track', self.onTrackReq)
        self.pan_ctrls.append(None)
        self.btn_plan = self.AddPanBtn(panel, 'Plan', self.onPlanReq)
        self.btn_go = self.AddPanBtn(panel, 'Go!', self.onGoReq)

        self.txt_cmd = wx.TextCtrl(panel, style=wx.TE_PROCESS_ENTER, value='{"C":"INFO"}')
        self.btn_cmd = wx.Button(panel, wx.ID_ANY, 'Send')

        self.unitPan = draw.UnitPanel(panel)
        #self.chart = draw.ChartPanel(panel)
        self.camera = camera.CameraPanel(panel)
        self.map = map.MapPanel(panel, self, self.model, "map.json", self.LogString, self.LogErrorString)
        self.controller=controller.Controller(self, self.model, self.map, self.LogString, self.LogErrorString)
        self.map.AddController(self.controller)

        self.btn_map_zoom_in = self.AddToolBtn(panel, '+', lambda evt, zoom='in': self.map.onZoom(evt, zoom))
        self.btn_map_zoom_out = self.AddToolBtn(panel, '-', lambda evt, zoom='out': self.map.onZoom(evt, zoom))
        self.btn_map_left = self.AddToolBtn(panel, u"\u25C0", lambda evt, move='left': self.map.onButtonMove(evt, move))
        self.btn_map_right = self.AddToolBtn(panel, u"\u25B6", lambda evt, move='right': self.map.onButtonMove(evt, move))
        self.btn_map_up = self.AddToolBtn(panel, u"\u25B2", lambda evt, move='up': self.map.onButtonMove(evt, move))
        self.btn_map_dn = self.AddToolBtn(panel, u"\u25BC", lambda evt, move='dn': self.map.onButtonMove(evt, move))
        self.btn_map_fit = self.AddToolBtn(panel, u"\u25AD", self.map.onFit)

        self.btn_map_ref = self.AddToolTogglePos(panel, u"\u2020", 0)
        self.btn_map_pos = self.AddToolTogglePos(panel, u"\u25C9", 1)
        self.btn_map_targ = self.AddToolTogglePos(panel, u"\u2605", 2)
        self.map_ctrls.append(None)
        self.btn_rot_left = self.AddToolBtn(panel, "<<", lambda evt, steer=-5: self.controller.reqSteer(steer))
        self.btn_rot_right = self.AddToolBtn(panel, ">>", lambda evt, steer=5: self.controller.reqSteer(steer))
        self.btn_mov_left = self.AddToolBtn(panel, u"\u2190", lambda evt, steer=-90: self.controller.reqSteer(steer))
        self.btn_mov_right = self.AddToolBtn(panel, u"\u2192", lambda evt, steer=90: self.controller.reqSteer(steer))
        self.btn_mov_up = self.AddToolBtn(panel, u"\u2191", lambda evt, dir=1: self.reqMove(dir))
        self.btn_mov_dn = self.AddToolBtn(panel, u"\u2193", lambda evt, dir=-1: self.reqMove(dir))
        self.btn_mov_stop = self.AddToolBtn(panel, u"\u2717", lambda evt, dir=0: self.reqMove(dir))
        #self.btn_mov_bear = self.AddToolBtn(panel, "B", lambda evt, bear=int(self.txt_mov_speed.GetValue()): self.controller.reqBearing(bear))
        self.btn_mov_bear = self.AddToolBtn(panel, "B", lambda evt: self.reqBearing())

        self.txt_mov_speed = wx.TextCtrl(panel, size=(28 * 2, 28))
        self.txt_mov_speed.SetValue("20")


        self.map_ctrls.append(self.txt_mov_speed)
        self.cb_dist_sim = wx.CheckBox(panel)
        self.cb_dist_sim.SetValue(False)
        self.Bind(wx.EVT_CHECKBOX, self.onDistSimEvent, self.cb_dist_sim)
        self.map_ctrls.append(self.cb_dist_sim)

        self.layout(panel)
        # redirect text here
        #sys.stdout=self.log
        #sys.stderr=self.log
        self.Bind(EVT_LOG_EVENT, self.onLogEvent)
        self.Bind(EVT_UPD_EVENT, self.onUpdEvent)
        self.Bind(EVT_UPD_STAT_EVENT, self.onUpdStatEvent)
        self.Bind(EVT_INFO_EVENT, self.onInfoEvent)
        self.Bind(EVT_ACT_EVENT, self.onActionEvent)

        self.Bind(wx.EVT_BUTTON, self.onSendCmd, self.btn_cmd)
        self.txt_cmd.Bind(wx.EVT_KEY_DOWN, self.onEnterCmdText)

        self.LogString("Local address is %s" % socket.gethostbyname(socket.gethostname()))
        self.map.Reset() # init particles filter and position
        self.history = self.model["CMD_HIST"]
        if len(self.history) > 0 :
            self.history_ptr=len(self.history)-1
            self.txt_cmd.SetValue(self.history[self.history_ptr])
        self.dist_sim = False

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
        sizer_logs = wx.BoxSizer(wx.HORIZONTAL)

        sizer.Add(sizer_pan, 2, wx.ALL|wx.EXPAND, 5)
        #sizer.Add(self.grid, 0, wx.ALL|wx.EXPAND, 5)
        sizer.Add(sizer_logs, 1, wx.ALL|wx.EXPAND, 5)
        #sizer.Add(self.log, 1, wx.ALL|wx.EXPAND, 5)
        sizer.Add(sizer_cmd, 0, wx.ALL|wx.EXPAND, 5)

        ##sizer_pan.Add(self.unitPan, 1, wx.ALL|wx.EXPAND, 5)
        sizer_charts.Add(self.unitPan, 1, wx.ALL|wx.EXPAND, border=0)
        #sizer_charts.Add(self.chart, 1, wx.ALL|wx.EXPAND, border=0)
        sizer_charts.Add(self.camera, 1, wx.ALL|wx.EXPAND, border=0)
        sizer_pan.Add(sizer_charts, 1, wx.ALL|wx.EXPAND, border=0)

        sizer_logs.Add(self.log, 2, wx.ALL|wx.EXPAND, 5)
        sizer_logs.Add(self.log_a, 1, wx.ALL|wx.EXPAND, 5)

        #sizer_pan.Add(self.map, 2, wx.ALL|wx.EXPAND, border=0)
        sizer_pan.Add(sizer_map, 2, wx.ALL|wx.EXPAND, border=0)
        sizer_map.Add(self.map, 5, wx.ALL|wx.EXPAND, border=0)

        sizer_map.Add(sizer_map_ctrls, 0, wx.ALL|wx.EXPAND, border=0)

        for bctrl in self.map_ctrls:
            if bctrl is not None:
                sizer_map_ctrls.Add(bctrl, 0, wx.LEFT | wx.BOTTOM, 0)
            else:
                sizer_map_ctrls.AddSpacer(24)

        sizer_pan.Add(sizer_ctrls, 0, wx.ALL|wx.RIGHT, 5)

        for bctrl in self.pan_ctrls:
            if bctrl is not None:
                sizer_ctrls.Add(bctrl, 0, wx.LEFT | wx.BOTTOM, 0)
            else:
                sizer_ctrls.AddSpacer(24)

        sizer_cmd.Add(self.txt_cmd, 10, wx.ALL|wx.CENTER, 5)
        sizer_cmd.Add(self.btn_cmd, 0, wx.ALL|wx.CENTER, 5)

        panel.SetSizer(sizer)
        #panel.SetAutoLayout(True)
        panel.Layout()
        sizer.Fit(panel)

    def AddPanBtn(self, panel, text, bind):
        but = wx.Button(panel, wx.ID_ANY, text)
        self.Bind(wx.EVT_BUTTON, bind, but)
        self.pan_ctrls.append(but)
        return but

    def AddToolBtn(self, panel, text, bind):
        but = wx.Button(panel, wx.ID_ANY, text, size=(28, 28))
        self.Bind(wx.EVT_BUTTON, bind, but)
        self.map_ctrls.append(but)
        return but

    def AddToolTogglePos(self, panel, text, toggle_code):
        but = wx.ToggleButton(panel, wx.ID_ANY, text, size=(28, 28))
        self.Bind(wx.EVT_TOGGLEBUTTON,
                  lambda evt: self.map.onPosToggle(evt, toggle_code, but.GetValue()) and self.ToggleGroup1(but),
                  but)
        self.map_ctrls.append(but)
        return but

    def ToggleGroup1(self, sel):
        group = [self.btn_map_ref, self.btn_map_pos, self.btn_map_targ]
        for but in group :
            if but != sel : but.SetValue(False)

    def AddLine(self, msg, color=None) :
        logging.info(msg)
        self.log.AddLine(msg, color)
        if color=='RED' :
            self.log_a.AddLine(msg, color)

    def LogString(self, message, color='BLACK') :
        event = LogEvent(msg=message, color=color)
        wx.PostEvent(self, event)

    def LogErrorString(self, message) :
        self.LogString(message, color='RED')
        #self.statusbar.SetStatusText(message, 2)

    def InfoString(self, message, color='BLACK') :
        event = InfoEvent(msg=message, color=color)
        wx.PostEvent(self, event)

    def UpdatePos(self, **kwargs) :
        event = UpdEvent(**kwargs)
        wx.PostEvent(self, event)

    def UpdateStatus(self, **kwargs) :
        event = UpdStatusEvent(**kwargs)
        wx.PostEvent(self, event)

    def ActionShow(self, action) :
        event = ActEvent(action=action)
        wx.PostEvent(self, event)

    def DispMousePos(self, pt):
        self.statusbar.SetStatusText('({}, {})'.format(int(pt[0]), int(pt[1])), 3)

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
        self.controller.reqResetMPU(action="MPU", pos=self.map.init_start)
        self.map.Reset()
        self.map.UpdateDrawing()

    def onResetMPUIntReq(self, event):
        self.controller.reqResetMPU(action="MPU_INT", pos=self.map.init_start) # reset integrator
        self.map.Reset()
        self.map.UpdateDrawing()

    def onScanReq(self, event):
        if self.controller.isScanning() :
            self.controller.stopScan()
        else :
            self.controller.startScan()

    def onTrackReq(self, event):
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
            if len(self.controller.planner.spath)<2 :
                self.LogErrorString('No path')
                return
            try :
                speed=int(self.txt_mov_speed.GetValue())
            except ValueError : speed=20
            #if self.controller.isSimulating():
            #    self.controller.stopSimulation()
            self.controller.startPathRunning(speed)

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

    def onInfoEvent(self, evt):
        self.statusbar.SetStatusText(evt.msg, 2)

    def onLogEvent(self, evt):
        self.AddLine(evt.msg, evt.color)

    def onUpdEvent(self, evt):
        #self.statusbar.SetStatusText(str(self.model["FHS"]), 0)
        #self.statusbar.SetStatusText("%(YPR)s" % self.model, 1)
        if evt.reset==True :
            self.chart.Reset()
            self.map.Reset()
        else :
            self.map.UpdateData()
            #if self.controller.isPathRunning() :
            #    self.controller.updatePathRunning()

        self.unitPan.UpdateData(self.model["T_ATT"], self.model["YPR"], self.model["V"], self.controller.unit.a_mean)
        self.camera.UpdateData(sensors=self.model["S"])
        #self.chart.UpdateData(self.model["T_ATT"], self.model["YPR"], self.model["V"])


    def onActionEvent(self, evt):
        if "C" in evt.action and evt.action["C"] in {"M", "D", "S", "B"} :
            self.unitPan.ShowAction(evt.action)
            #self.AddLine(evt.action["C"], 'orange')


    def onUpdStatEvent(self, evt):
        self.statusbar.SetStatusText(str(self.model["FHS"]), 0)

    def onDistSimEvent(self, evt):
        self.dist_sim = self.cb_dist_sim.GetValue()
        self.map.dist_sim = self.dist_sim
        self.map.UpdateDrawing()

    def reqMove(self, dir):
        try :
            speed=int(self.txt_mov_speed.GetValue())
        except ValueError : speed=20
        self.controller.reqMoveSpeed(speed*dir)

    def reqBearing(self):
        try :
            ang=int(self.txt_mov_speed.GetValue())
        except ValueError : ang=0
        self.controller.reqBearing(ang)

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

class LogControl(wx.TextCtrl):
    LOG_LINES = 36
    def __init__(self, parent, sz):
        wx.TextCtrl.__init__(self, parent, wx.ID_ANY, size=sz, style=wx.TE_MULTILINE|wx.TE_READONLY|wx.HSCROLL|wx.TE_RICH)
        self.bg=self.GetBackgroundColour()
        self.logcnt=0
    def AddLine(self, msg, color=None) :
        while self.GetNumberOfLines()>self.LOG_LINES:
            self.Remove(0, self.GetLineLength(0)+1)
        if color is None : color= wx.BLACK
        self.SetDefaultStyle(wx.TextAttr(color,self.bg))
        self.AppendText(msg)
        if not msg.endswith('\n'):
            self.AppendText('\n')
        self.logcnt=self.logcnt+1


# Run the program
if __name__ == "__main__":
    logging.basicConfig(filename='console.log',level=logging.INFO)
    app = wx.App(False)
    frame = MyForm().Show()
    app.MainLoop()

