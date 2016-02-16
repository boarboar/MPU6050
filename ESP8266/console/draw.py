import wx
import math

class UnitPanel(wx.Window):
    " draw panel with double buffering"
    UNIT_WIDTH=80
    UNIT_HEIGHT=100
    UNIT_ARROW_SIZE=10
    def __init__(self, parent):
        wx.Window.__init__(self, parent, wx.ID_ANY, style=wx.SIMPLE_BORDER, size=(240,240))
        self.Bind(wx.EVT_PAINT, self.OnPaint)
        wx.EVT_SIZE(self, self.OnSize)
        self.att=[0,0,0]
        self.yaw_sin=0.0
        self.yaw_cos=1.0
        # in real coords
        self.shape=[wx.Point(-self.UNIT_WIDTH/2, -self.UNIT_HEIGHT/2),
                    wx.Point(-self.UNIT_WIDTH/2, self.UNIT_HEIGHT/2),
                    wx.Point(self.UNIT_WIDTH/2, self.UNIT_HEIGHT/2),
                    wx.Point(self.UNIT_WIDTH/2, -self.UNIT_HEIGHT/2),
                    ]
        yl=self.UNIT_HEIGHT*3/4
        self.y_line=[wx.Point(0,0), wx.Point(0,yl),
                     wx.Point(self.UNIT_ARROW_SIZE/2,yl-self.UNIT_ARROW_SIZE/2),
                     wx.Point(0,yl),
                     wx.Point(-self.UNIT_ARROW_SIZE/2,yl-self.UNIT_ARROW_SIZE/2)
                     ]
        self.OnSize(None)

    def OnSize(self,event):
        w, h = self.GetSize()
        self.x0=w/2
        self.y0=h/2
        Size  = self.ClientSize
        self._Buffer = wx.EmptyBitmap(*Size)
        self.UpdateDrawing()

    def OnPaint(self, event=None):
        dc = wx.BufferedPaintDC(self, self._Buffer)

    def UpdateDrawing(self) :
        dc = wx.MemoryDC()
        dc.SelectObject(self._Buffer)
        self.Draw(dc)
        del dc # need to get rid of the MemoryDC before Update() is called.
        self.Refresh()
        self.Update()

    def UpdateData(self, att):
        self.att=att
        self.yaw_cos=math.cos(att[0]*math.pi/180.0)
        self.yaw_sin=math.sin(att[0]*math.pi/180.0)
        self.UpdateDrawing()

    def Draw(self, dc):
        dc.Clear()
        dc.SetPen(wx.Pen(wx.BLUE, 4))
        dc.DrawPolygon(self.ts(self.shape))
        dc.SetPen(wx.Pen(wx.GREEN, 2))
        dc.DrawLines(self.ts(self.y_line))

    def ts(self, pts):
        return [self.tp(p) for p in pts]

    def tp(self, p):
        x, y =p.Get()
        x1=x*self.yaw_cos+y*self.yaw_sin
        y1=-x*self.yaw_sin+y*self.yaw_cos
        return wx.Point(x1+self.x0,-y1+self.y0)

class DrawPanel(wx.Window):
    " draw panel"
    def __init__(self, parent):
        wx.Window.__init__(self, parent, wx.ID_ANY, style=wx.SIMPLE_BORDER, size=(240,240))
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

    def AddPoint(self, x, y):
        self.points.append((x, y))

        #print self.points

        dc = wx.ClientDC(self)
        dc.DrawCheckMark(x, y, 10, 10)
        dc.SetPen(wx.Pen(wx.BLACK, 4))
