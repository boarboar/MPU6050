import wx

class DrawPanel(wx.Window):
    " draw panel"
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
