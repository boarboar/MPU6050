import wx
import math

class UnitPanel(wx.Window):
    " draw panel with double buffering"
    UNIT_WIDTH=80
    UNIT_HEIGHT=100
    UNIT_ARROW_SIZE=10
    V_SCALE=100  # 1 cm/s = 0.01 m/s * 1000 = 1 pix
    def __init__(self, parent):
        wx.Window.__init__(self, parent, wx.ID_ANY, style=wx.SIMPLE_BORDER, size=(160,160))
        self.SetBackgroundColour(wx.WHITE)
        self.Bind(wx.EVT_PAINT, self.OnPaint)
        self.Bind(wx.EVT_SIZE, self.OnSize)
        self.att=[0,0,0]
        self.t=0
        self.v=[0,0,0]
        self.r_sin=0.0
        self.r_cos=1.0
        # in real coords
        self.shape=[wx.Point(-self.UNIT_WIDTH/2, -self.UNIT_HEIGHT/2),
                    wx.Point(-self.UNIT_WIDTH/2, self.UNIT_HEIGHT/2),
                    wx.Point(0, self.UNIT_HEIGHT*3/5),
                    wx.Point(self.UNIT_WIDTH/2, self.UNIT_HEIGHT/2),
                    wx.Point(self.UNIT_WIDTH/2, -self.UNIT_HEIGHT/2),
                    ]
        self.y_line=self.MakeArrow(self.UNIT_HEIGHT*3/4)
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

    def UpdateData(self, t, att, v=None):
        self.t=t
        self.att=att
        self.v=v
        self.UpdateDrawing()

    def Draw(self, dc):
        dc.SetBackground(wx.Brush(wx.WHITE))
        dc.Clear()
        dc.SetTextForeground(wx.BLACK)
        dc.SetTextBackground(wx.WHITE)
        dc.SetPen(wx.Pen(wx.BLUE, 4))
        self.SetRotation(self.att[0]*math.pi/180.0)
        dc.DrawPolygon(self.ts(self.shape))
        dc.SetPen(wx.Pen(wx.GREEN, 2))
        dc.DrawLines(self.ts(self.y_line))
        dc.DrawText(str(self.t/1000), self.x0*2-50, 5)
        vv = math.hypot(self.v[0], self.v[1])*self.V_SCALE
        if vv>1 :
            self.SetRotation(math.atan2(self.v[0], self.v[1]))
            dc.SetPen(wx.Pen(wx.RED, 4))
            dc.DrawLines(self.ts(self.MakeArrow(vv)))
        # todo - vertical
        # Zx=cos(y)*sin(p)*cos(r)+sin(y)*sin(r)
        # Zy=sin(y)*sin(p)*cos(r)-cos(y)*sin(r)

    def MakeArrow(self, len):
        return [wx.Point(0,0), wx.Point(0,len),
                     wx.Point(self.UNIT_ARROW_SIZE/2,len-self.UNIT_ARROW_SIZE/2),
                     wx.Point(0,len),
                     wx.Point(-self.UNIT_ARROW_SIZE/2,len-self.UNIT_ARROW_SIZE/2)
                     ]

    def SetRotation(self, angle):
        self.r_cos=math.cos(angle)
        self.r_sin=math.sin(angle)

    def ts(self, pts):
        return [self.tp(p) for p in pts]

    def tp(self, p):
        x, y = p.Get()
        x1=x*self.r_cos+y*self.r_sin
        y1=-x*self.r_sin+y*self.r_cos
        return wx.Point(x1+self.x0,-y1+self.y0)

#
#
#

class ChartPanel(wx.Window):
    " draw panel"
    SCALE_MSEC=2.0/1000.0
    def __init__(self, parent):
        wx.Window.__init__(self, parent, wx.ID_ANY, style=wx.SIMPLE_BORDER, size=(240,240))
        self.SetBackgroundColour('BLACK')
        self.Bind(wx.EVT_PAINT, self.OnPaint)
        self.Bind(wx.EVT_SIZE, self.OnSize)
        self.points=[] #(t, att[y,p,r], v3d_cart[x,y,z], v2d_pol[fi,r])
        self.t0=0

    def OnSize(self,event):
        w, h = self.GetSize()
        self.w=w
        self.h=h
        self.y_scale=h/360.0
        self.UpdateDrawing()

    def OnPaint(self, event=None):
        dc = wx.PaintDC(self)
        dc.SetBackground(wx.Brush(wx.BLACK))
        dc.Clear()
        dc.SetPen(wx.Pen(wx.WHITE, 1))
        dc.SetTextForeground(wx.WHITE)
        dc.SetTextBackground(wx.BLACK)
        # t-grid
        t=(self.t0/60000)*60000
        if t<self.t0 : t = t+60000
        x = (t-self.t0)*self.SCALE_MSEC
        while x<self.w :
            dc.DrawLine(x, 0, x, self.h)
            dc.DrawText(str(t/1000), x+5, self.h/2+5)
            x = x+60000*self.SCALE_MSEC
            t=t+60000

        dc.DrawLine(0, self.h/2, self.w, self.h/2)
        dc.SetPen(wx.Pen(wx.WHITE, 1, style=wx.PENSTYLE_SHORT_DASH))
        dc.DrawLine(0, self.h/4, self.w, self.h/4) #+90
        dc.DrawLine(0, self.h*3/4, self.w, self.h*3/4) #-90

        if len(self.points)>1 :
            point0=None
            for point in self.points:
                x1=(point[0]-self.t0)*self.SCALE_MSEC
                if x1 > self.w : break
                if point0 != None:
                    x0=(point0[0]-self.t0)*self.SCALE_MSEC
                    if x0>=0 :
                        dc.SetPen(wx.Pen(wx.GREEN, 2))
                        dc.DrawLine(x0, self.h/2-point0[1][0]*self.y_scale, x1, self.h/2-point[1][0]*self.y_scale)
                        dc.SetPen(wx.Pen(wx.RED, 2))
                        dc.DrawLine(x0, self.h/2-point0[3][0]*self.y_scale, x1, self.h/2-point[3][0]*self.y_scale)
                point0=point


    def UpdateDrawing(self) :
        self.Refresh()
        self.Update()

    def UpdateData(self, t, att, v=None):
        if len(self.points)==0 : self.t0=0
        point=(t, att, v, (math.atan2(v[0], v[1])*180.0/math.pi, math.hypot(v[0], v[1])))
        self.points.append(point)
        x1=(point[0]-self.t0)*self.SCALE_MSEC
        if x1 > self.w :
            # split in half
            l = len(self.points)
            if l>1 :
                self.points = self.points[l/2:]
                self.t0=self.points[0][0]
                self.UpdateDrawing()
        elif len(self.points)>1:
            dc = wx.ClientDC(self)
            point0=self.points[-2]
            x0=(point0[0]-self.t0)*self.SCALE_MSEC
            if x0>=0 :
                dc.SetPen(wx.Pen(wx.GREEN, 2))
                dc.DrawLine(x0, self.h/2-point0[1][0]*self.y_scale, x1, self.h/2-point[1][0]*self.y_scale)
                dc.SetPen(wx.Pen(wx.RED, 2))
                dc.DrawLine(x0, self.h/2-point0[3][0]*self.y_scale, x1, self.h/2-point[3][0]*self.y_scale)

#
#
#

class DrawPanel(wx.Window):
    " draw panel"
    def __init__(self, parent):
        wx.Window.__init__(self, parent, wx.ID_ANY, style=wx.SIMPLE_BORDER, size=(240,240))
        self.Bind(wx.EVT_PAINT, self.OnPaint)
        self.points=[(0,0), (50, 50)]

    def OnPaint(self, event=None):
        dc = wx.PaintDC(self)
        dc.Clear()
        dc.SetPen(wx.Pen(wx.BLACK, 1))
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
