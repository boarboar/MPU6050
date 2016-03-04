import wx
import json
import sys
import math
import model

from pprint import pprint

class MapPanel(wx.Window):
    " MAP panel, with doublebuffering"
    UNIT_WIDTH=18
    UNIT_HEIGHT=30
    def __init__(self, parent, model, mapfile):
        wx.Window.__init__(self, parent, wx.ID_ANY, style=wx.SIMPLE_BORDER, size=(240,240))
        self.Bind(wx.EVT_PAINT, self.OnPaint)
        self.Bind(wx.EVT_SIZE, self.OnSize)
        self.__model=model
        self.__map=[]
        self.__x0=0
        self.__y0=0
        self.__scale=1
        self.__r_sin=0.0
        self.__r_cos=1.0
        self.__r_x=0
        self.__r_y=0
        self.__boundRect=[sys.maxint, sys.maxint, -sys.maxint, -sys.maxint]
        try:
            with open(mapfile) as data_file:
                self.__map = json.load(data_file)
            #pprint(__map)
            for area in self.__map["AREAS"] :
                for wall in area["WALLS"] :
                    self.adjustBound(wall["C"][0], wall["C"][1])
                    self.adjustBound(wall["C"][2], wall["C"][3])
            self.__xu0, self.__yu0=self.__map["START"]
            print("Map loaded")
            print(self.__boundRect)
        #except IOError: pass
        except : pass
        self.__shape=[wx.Point(-self.UNIT_WIDTH/2, -self.UNIT_HEIGHT/2),
                    wx.Point(-self.UNIT_WIDTH/2, self.UNIT_HEIGHT/2),
                    wx.Point(0, self.UNIT_HEIGHT*3/5),
                    wx.Point(self.UNIT_WIDTH/2, self.UNIT_HEIGHT/2),
                    wx.Point(self.UNIT_WIDTH/2, -self.UNIT_HEIGHT/2),
                    ]
        self.OnSize(None)

    def OnSize(self,event):
        Size  = self.ClientSize
        self.__x0=Size.width/2
        self.__y0=Size.height/2
        w = (self.__boundRect[2]-self.__boundRect[0])*1.1
        h = (self.__boundRect[3]-self.__boundRect[1])*1.1
        if w>0 :
            self.__scale=Size.width/w
        if h>0 and Size.height/h<self.__scale:
            self.__scale=Size.height/h
        self._Buffer = wx.EmptyBitmap(*Size)
        self.UpdateDrawing()

    def OnPaint(self, event):
        #dc = wx.PaintDC(self)
        #self.Draw(dc)
        dc = wx.BufferedPaintDC(self, self._Buffer)

    def Draw(self, dc):
        dc.Clear()
        wall_pen=wx.Pen(wx.BLACK, 4)
        door_pen=wx.Pen("GRAY", 4)
        dc.SetBackgroundMode(wx.TRANSPARENT)
        dc.SetBrush(wx.TRANSPARENT_BRUSH)
        dc.SetTextForeground(wx.BLACK)
        dc.SetTextBackground(wx.WHITE)

        try:
            for area in self.__map["AREAS"] :
                for wall in area["WALLS"] :
                    if wall["T"]=="W" :
                        dc.SetPen(wall_pen)
                    else :
                        dc.SetPen(door_pen)
                    dc.DrawLinePoint(self.tc(wall["C"][0],wall["C"][1]),self.tc(wall["C"][2],wall["C"][3]))
            zero = self.tc(self.__xu0, self.__yu0)
            dc.SetPen(wx.Pen(wx.BLACK, 1))
            dc.DrawLine(zero.x-10, zero.y, zero.x+10, zero.y)
            dc.DrawLine(zero.x, zero.y-10, zero.x, zero.y+10)
            zero = self.tc(0, 0)
            dc.DrawTextPoint("(0,0)", zero)

            dc.SetPen(wx.Pen(wx.BLUE, 1))
            dc.DrawPolygon(self.tsu(self.__shape))

        except KeyError : pass
        except IndexError : pass

    def UpdateDrawing(self) :
        dc = wx.MemoryDC()
        dc.SelectObject(self._Buffer)
        self.Draw(dc)
        del dc # need to get rid of the MemoryDC before Update() is called.
        self.Refresh()
        self.Update()

    def UpdateData(self):
        try:
            yaw, pitch, roll = [a*math.pi/180.0 for a in self.__model["YPR"]]
            x, y, z = [int(a*100) for a in self.__model["CRD"]]
            self.SetUnitPos(x, y, yaw)
        except KeyError : pass
        except IndexError : pass
        self.UpdateDrawing()

    def SetUnitPos(self, x, y, angle):
        self.__r_cos=math.cos(angle)
        self.__r_sin=math.sin(angle)
        self.__r_x=x
        self.__r_y=y

    def adjustBound(self, x, y):
        if x<self.__boundRect[0] : self.__boundRect[0]=x
        if y<self.__boundRect[1] : self.__boundRect[1]=y
        if x>self.__boundRect[2] : self.__boundRect[2]=x
        if y>self.__boundRect[3] : self.__boundRect[3]=y

    def tc(self, x, y):
        x1=(x-(self.__boundRect[2]+self.__boundRect[0])/2)*self.__scale
        y1=(y-(self.__boundRect[3]+self.__boundRect[1])/2)*self.__scale
        return wx.Point(x1+self.__x0,-y1+self.__y0)

    def tsu(self, pts):
        return [self.tpu(p) for p in pts]

    def tpu(self, p):
        x, y = p.Get()
        x1=x*self.__r_cos+y*self.__r_sin+self.__r_x+self.__xu0
        y1=-x*self.__r_sin+y*self.__r_cos+self.__r_y+self.__yu0
        return self.tc(x1,y1)