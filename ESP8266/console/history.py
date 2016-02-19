import wx
import wx.grid

class HistoryDialog(wx.Dialog):
    def __init__(self, parent, model, *args, **kw):
        wx.Dialog.__init__(self, None, -1, 'Dialog Subclass')
        self.model = model
        self.data=self.model["MHIST"][1]
        self.InitUI()
        self.SetSize((360, 360))
        self.SetTitle("Measurements")

    def InitUI(self):
        cols=7
        self.grid = wx.grid.Grid(self, -1, style=wx.HSCROLL|wx.VSCROLL)
        self.grid.CreateGrid(len(self.data), cols)
        self.grid.SetColLabelValue(0, "T")
        self.grid.SetColLabelValue(1, "A.X")
        self.grid.SetColLabelValue(2, "A.Y")
        self.grid.SetColLabelValue(3, "A.Z")
        self.grid.SetColLabelValue(4, "Y")
        self.grid.SetColLabelValue(5, "P")
        self.grid.SetColLabelValue(6, "R")
        self.grid.SetRowLabelSize(0)

        for i in range(cols):
            self.grid.SetColSize(i, 40)

        row=0
        for item in self.data :
            try:
                self.grid.SetCellValue(row, 0, str(item["T"]))
                self.grid.SetCellValue(row, 1, str(item["A"][0]))
                self.grid.SetCellValue(row, 2, str(item["A"][1]))
                self.grid.SetCellValue(row, 3, str(item["A"][2]))
                self.grid.SetCellValue(row, 4, str(item["YPR"][0]))
                self.grid.SetCellValue(row, 5, str(item["YPR"][1]))
                self.grid.SetCellValue(row, 6, str(item["YPR"][2]))
            except KeyError: pass
            row=row+1

        #self.grid.SetMaxSize((480, 240))


        #self.grid.SetCellValue(0, 0, "val")

        closeButton = wx.Button(self, label='Close')

        vbox = wx.BoxSizer(wx.VERTICAL)
        vbox.Add(self.grid, flag=wx.CENTER|wx.EXPAND, border=15)
        vbox.Add(closeButton, flag=wx.CENTER, border=15)
        self.SetSizer(vbox)
        self.Layout()
        vbox.Fit(self)

        closeButton.Bind(wx.EVT_BUTTON, self.OnClose)

    def OnClose(self, e):
        self.SetReturnCode(False)
        self.Destroy()