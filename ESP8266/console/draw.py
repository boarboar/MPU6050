import wx
import math

class UnitPanel(wx.Window):
    " draw panel with double buffering"
    UNIT_WIDTH=85
    UNIT_HEIGHT=110
    UNIT_LEVEL_RAD=36
    UNIT_ARROW_SIZE=10
    UNIT_POW_HEIGHT = UNIT_HEIGHT/2+UNIT_HEIGHT/4
    UNIT_POW_WIDTH = UNIT_WIDTH/6

    V_SCALE=100  # 1 cm/s = 0.01 m/s * 1000 = 1 pix
    def __init__(self, parent):
        wx.Window.__init__(self, parent, wx.ID_ANY, style=wx.SIMPLE_BORDER, size=(160,160))
        self.SetBackgroundColour(wx.WHITE)
        self.Bind(wx.EVT_PAINT, self.OnPaint)
        self.Bind(wx.EVT_SIZE, self.OnSize)
        self.att=[0,0,0]
        self.a_loc=0
        self.t=0
        #self.v=[0,0,0]
        self.v = 0
        self.r_sin=0.0
        self.r_cos=1.0
        self.action=None
        self.pow=None
        # in real coords
        self.shape=[wx.Point(-self.UNIT_WIDTH/2, -self.UNIT_HEIGHT/2),
                    wx.Point(-self.UNIT_WIDTH/2, self.UNIT_HEIGHT/2),
                    wx.Point(0, self.UNIT_HEIGHT*3/5),
                    wx.Point(self.UNIT_WIDTH/2, self.UNIT_HEIGHT/2),
                    wx.Point(self.UNIT_WIDTH/2, -self.UNIT_HEIGHT/2),
                    ]
        self.axe_line=self.MakeArrow(self.UNIT_HEIGHT*3/4)
        self.OnSize(None)

    def OnSize(self,event):
        Size  = self.ClientSize
        self.x0=Size.width/2
        self.y0=Size.height/2
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

    def UpdateData(self, t, att, v=None, a_loc=None, pow=None):
        self.t=t
        self.att=att
        self.v=v
        if a_loc is not None: self.a_loc=a_loc
        self.action = None
        self.pow=pow
        self.UpdateDrawing()

    def ShowAction(self, action):
        self.action = action
        self.UpdateDrawing()

    def Draw(self, dc):
        #dc.SetBackground(wx.Brush(wx.WHITE))
        dc.Clear()
        dc.SetBackgroundMode(wx.TRANSPARENT)
        dc.SetBrush(wx.TRANSPARENT_BRUSH)
        dc.SetTextForeground(wx.BLACK)
        dc.SetTextBackground(wx.WHITE)
        dc.DrawText(str(self.t/1000), self.x0*2-50, 5)
        dc.SetPen(wx.Pen('GRAY', 2))
        yaw, pitch, roll = [a*math.pi/180.0 for a in self.att]
        self.SetRotation(yaw)
        dc.DrawPolygon(self.ts(self.shape))
        #dc.SetPen(wx.Pen(wx.BLACK, 2))
        #dc.SetTextForeground(wx.BLACK)
        dc.DrawLines(self.ts(self.axe_line)) ## X
        dc.DrawTextPoint("X", self.tp(self.axe_line[1]))
        self.SetRotation(yaw-math.pi*0.5)
        dc.DrawLines(self.ts(self.axe_line)) ## Y
        dc.DrawTextPoint("Y", self.tp(self.axe_line[1]))
        dc.DrawCirclePoint(self.tp(wx.Point(0, 0)), self.UNIT_LEVEL_RAD)
        """
        vv = math.hypot(self.v[0]/10, self.v[1]/10)*self.V_SCALE
        if vv>1 :
            #self.SetRotation(math.atan2(-self.v[1], self.v[0]))
            self.SetRotation(math.atan2(self.v[0], self.v[1]))
            dc.SetPen(wx.Pen(wx.RED, 2))
            dc.DrawLines(self.ts(self.MakeArrow(vv)))
            """
        # draw - vertical
        # Zx=cos(y)*sin(p)*cos(r)+sin(y)*sin(r)
        # Zy=sin(y)*sin(p)*cos(r)-cos(y)*sin(r)
        # Zz=cos(p)*cos(r)
        spcr=math.sin(pitch)*math.cos(roll);
        zx=math.cos(yaw)*spcr+math.sin(yaw)*math.sin(roll)
        zy=math.sin(yaw)*spcr-math.cos(yaw)*math.sin(roll)
        vv = math.hypot(zx, zy)*self.UNIT_LEVEL_RAD
        if vv>2 :
            line=[wx.Point(0, 0), wx.Point(0, vv)]
            self.SetRotation(math.atan2(zy, zx))
            dc.SetPen(wx.Pen("SALMON", 4))
            dc.DrawLines(self.ts(line))

        # localized
        dc.SetTextForeground(wx.BLUE)
        dc.SetPen(wx.Pen(wx.BLUE, 4))
        self.SetRotation(self.a_loc)
        dc.DrawPolygon(self.ts(self.shape))

        dc.SetPen(wx.Pen(wx.BLUE, 4))
        dc.DrawLines(self.ts(self.axe_line))  ## X
        dc.DrawTextPoint("X", self.tp(self.axe_line[1]))
        self.SetRotation(self.a_loc - math.pi * 0.5)
        dc.DrawLines(self.ts(self.axe_line))  ## Y
        dc.DrawTextPoint("Y", self.tp(self.axe_line[1]))

        self.SetRotation(self.a_loc)

        if self.v is not None and self.v>1:
            move_arrow = self.MakeArrow(self.v * self.UNIT_HEIGHT / 100)
            dc.SetPen(wx.Pen("GREEN", 4))
            dc.DrawLines(self.ts(move_arrow))

        if self.action is not None and 'C' in self.action:
            adeg = self.a_loc/math.pi*180.0
            if self.action['C']=='M' :
                move_arrow = self.MakeArrow(self.action['V']*self.UNIT_HEIGHT/100)
                dc.SetPen(wx.Pen("RED", 6))
                dc.DrawLines(self.ts(move_arrow))
                pass
            elif self.action['C']=='S':
                dc.SetPen(wx.Pen("RED", 6))
                a = self.action['S'] #steering angle
                #left = self.tp(wx.Point(-self.UNIT_WIDTH/2, self.UNIT_WIDTH/2))
                dc.DrawEllipticArc(self.x0-self.UNIT_WIDTH/2, self.y0-self.UNIT_WIDTH/2, self.UNIT_WIDTH, self.UNIT_WIDTH, 90-adeg, 90-a-adeg)
            elif self.action['C'] == 'B':
                dc.SetPen(wx.Pen("GREEN", 6))
                a = self.action['A']  #absolute steering angle
                #left = self.tp(wx.Point(-self.UNIT_WIDTH / 2, self.UNIT_WIDTH / 2))
                dc.DrawEllipticArc(self.x0-self.UNIT_WIDTH/2, self.y0-self.UNIT_WIDTH/2, self.UNIT_WIDTH, self.UNIT_WIDTH, 90-adeg, 90 - a)

        if self.pow is not None:
            self.DrawPower(dc, 0)
            self.DrawPower(dc, 1)

    def DrawPower(self, dc, pos):
        dc.SetPen(wx.Pen("BLACK", 2))
        if pos==0:
            x=self.UNIT_POW_WIDTH
        else:
            x =(self.x0-self.UNIT_POW_WIDTH)*2
        y=self.y0
        dc.SetBrush(wx.Brush("WHITE", wx.TRANSPARENT))
        dc.DrawRectangle(x, y, self.UNIT_POW_WIDTH, self.UNIT_POW_HEIGHT)
        ph=int(self.pow[pos]/100.0*(self.UNIT_POW_HEIGHT-4))
        if ph>=0 :
            dc.SetPen(wx.Pen("GREEN", 2))
            dc.SetBrush(wx.Brush("GREEN", wx.SOLID))
            dc.DrawRectangle(x+2, y+self.UNIT_POW_HEIGHT-2-ph, self.UNIT_POW_WIDTH-4, ph)
        else:
            dc.SetPen(wx.Pen("RED", 2))
            dc.SetBrush(wx.Brush("RED", wx.SOLID))
            dc.DrawRectangle(x + 2, y + +2, self.UNIT_POW_WIDTH - 4, -ph)

    def MakeArrow(self, len):
        shape=None
        if len==0 :
            shape = [wx.Point(self.UNIT_ARROW_SIZE/2,self.UNIT_ARROW_SIZE/2),
                     wx.Point(-self.UNIT_ARROW_SIZE / 2, -self.UNIT_ARROW_SIZE / 2),
                     wx.Point(0, 0),
                     wx.Point(-self.UNIT_ARROW_SIZE / 2, self.UNIT_ARROW_SIZE / 2),
                     wx.Point(self.UNIT_ARROW_SIZE / 2, -self.UNIT_ARROW_SIZE / 2)]

        elif len>0:
            shape = [wx.Point(0,0), wx.Point(0,len),
                     wx.Point(self.UNIT_ARROW_SIZE/2,len-self.UNIT_ARROW_SIZE/2),
                     wx.Point(0,len),
                     wx.Point(-self.UNIT_ARROW_SIZE/2,len-self.UNIT_ARROW_SIZE/2)
                     ]
        else :
            shape = [wx.Point(0, 0), wx.Point(0, len),
                     wx.Point(self.UNIT_ARROW_SIZE / 2, len + self.UNIT_ARROW_SIZE / 2),
                     wx.Point(0, len),
                     wx.Point(-self.UNIT_ARROW_SIZE / 2, len + self.UNIT_ARROW_SIZE / 2)
                     ]

        return shape


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
    SCALE_MSEC=5.0/1000.0
    def __init__(self, parent):
        wx.Window.__init__(self, parent, wx.ID_ANY, style=wx.SIMPLE_BORDER, size=(160,120))
        self.SetBackgroundColour('BLACK')
        self.Bind(wx.EVT_PAINT, self.OnPaint)
        self.Bind(wx.EVT_SIZE, self.OnSize)
        self.points=[] #(t, att[y,p,r], v3d_cart[x,y,z], v2d_pol[fi,r])
        self.t0=0

    def OnSize(self,event):
        Size  = self.ClientSize
        self.w=Size.width
        self.h=Size.height
        self.y_scale=self.h/360.0
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
        #point=(t, att, v, (math.atan2(-v[1], v[0])*180.0/math.pi, math.hypot(v[0], v[1])))
        v= [0 , 0] # tempoarrily
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

    def Reset(self):
        self.points[:] = []
        self.t0=0
        self.UpdateDrawing()
