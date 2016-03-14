import wx
import math
import json
import sys
from unitmap import UnitMap
import model

from pprint import pprint

class MapPanel(wx.Window, UnitMap):
    " MAP panel, with doublebuffering"
    UNIT_WIDTH=18
    UNIT_HEIGHT=30
    def __init__(self, parent, model, mapfile):
        wx.Window.__init__(self, parent, wx.ID_ANY, style=wx.SIMPLE_BORDER, size=(240,240))
        UnitMap.__init__(self, mapfile)
        self.Bind(wx.EVT_PAINT, self.OnPaint)
        self.Bind(wx.EVT_SIZE, self.OnSize)
        self.__model=model
        self.__map=[]
        self.__x0, self.__y0 = (0, 0) # canvas center
        self.__scale=1
        #self.__inside=False
        #self.__scans=[-1,-1,-1]
        self.__shape=[wx.Point(-self.UNIT_WIDTH/2, -self.UNIT_HEIGHT/2),
                    wx.Point(-self.UNIT_WIDTH/2, self.UNIT_HEIGHT/2),
                    wx.Point(0, self.UNIT_HEIGHT*3/5),
                    wx.Point(self.UNIT_WIDTH/2, self.UNIT_HEIGHT/2),
                    wx.Point(self.UNIT_WIDTH/2, -self.UNIT_HEIGHT/2),
                    ]
        self.InitParticles()
        self.OnSize(None)

    def OnSize(self,event):
        Size  = self.ClientSize
        self.__x0=Size.width/2
        self.__y0=Size.height/2
        w = (self.boundRect[2]-self.boundRect[0])*1.1
        h = (self.boundRect[3]-self.boundRect[1])*1.1
        if w>0 :
            self.__scale=Size.width/w
        if h>0 and Size.height/h<self.__scale:
            self.__scale=Size.height/h
        self._Buffer = wx.EmptyBitmap(*Size)
        self.UpdateDrawing()

    def OnPaint(self, event):
        dc = wx.BufferedPaintDC(self, self._Buffer)

    def Draw(self, dc):
        dc.Clear()
        wall_pen=wx.Pen(wx.BLACK, 4)
        door_pen=wx.Pen("GRAY", 4)
        dc.SetBackgroundMode(wx.TRANSPARENT)
        dc.SetBrush(wx.TRANSPARENT_BRUSH)
        dc.SetTextForeground(wx.BLACK)
        dc.SetTextBackground(wx.WHITE)
        ray_pen=wx.Pen(wx.BLACK, 1, wx.PENSTYLE_LONG_DASH)
        p_ray_pen=wx.Pen("GRAY", 1, wx.PENSTYLE_SHORT_DASH)

        #draw map

        try:
            for area in self.map["AREAS"] :
                for wall in area["WALLS"] :
                    if wall["T"]=="W" :
                        dc.SetPen(wall_pen)
                    else :
                        dc.SetPen(door_pen)
                    dc.DrawLinePoint(self.tc(wall["C"][0],wall["C"][1]),self.tc(wall["C"][2],wall["C"][3]))
            zero = self.tc(self.xu0, self.yu0)
            dc.SetPen(wx.Pen(wx.BLACK, 1))
            dc.DrawLine(zero.x-10, zero.y, zero.x+10, zero.y)
            dc.DrawLine(zero.x, zero.y-10, zero.x, zero.y+10)
            zero = self.tc(0, 0)
            dc.DrawTextPoint("(0,0)", zero)
        except KeyError : pass
        except IndexError : pass

        #draw particles

        c_pen=wx.Pen(wx.RED, 1)
        c_pen_0=wx.Pen("GRAY", 1)
        a_pen=wx.Pen(wx.BLACK, 2)
        dc.SetBackgroundMode(wx.SOLID)
        dc.SetBrush(wx.RED_BRUSH)

        for p in self.particles :
            rad=1+math.log10(1+p.w*10)*8
            #rad=5
            c=self.tc(p.x,p.y)
            if rad>1 : dc.SetPen(c_pen)
            else : dc.SetPen(c_pen_0)
            dc.DrawCirclePoint(c, rad)
            ca=wx.Point(c.x+10*math.sin(p.a), c.y-10*math.cos(p.a))
            #dc.SetPen(a_pen)
            dc.DrawLinePoint(c, ca)
            continue
            # this staff below is for test purposes, skip it
            dc.SetPen(p_ray_pen)
            rays=self.getParticleRays(p)
            for r in rays:
                cr=self.tc(r[1][0], r[1][1])
                dc.DrawLinePoint(c, cr)
            ints=self.getParticleIntersects(p)
            dc.SetPen(wx.Pen(wx.BLUE, 1))
            for i in ints:
                if i != None :
                    c=self.tc(i[0], i[1])
                    dc.DrawCirclePoint(c, 5)


        dc.SetBackgroundMode(wx.TRANSPARENT)
        dc.SetBrush(wx.TRANSPARENT_BRUSH)

        # draw estimation and robot

        mx, my, var = self.getMeanDistribution()
        c=self.tc(mx,my)
        dc.SetPen(wx.Pen(wx.BLACK, 1))
        dc.DrawCirclePoint(c, var*self.__scale)

        if self.isInside :
            dc.SetPen(wx.Pen(wx.BLUE, 2))
        else :
            dc.SetPen(wx.Pen(wx.RED, 2))

        dc.DrawPolygon(self.tsu(self.__shape))
        # draw sensor rays
        if self.isInside :
            i=0
            inters_pen=wx.Pen(wx.GREEN, 2)
            for ray in self.scan_rays:
                dc.SetPen(ray_pen)
                dc.DrawLinePoint(self.tpu(wx.Point(0, 0)), self.tpu(wx.Point(ray[0]*500, ray[1]*500)))
                idist=self.scans[i]
                i=i+1
                if idist>0 :
                    dc.SetPen(inters_pen)
                    intrs = self.tpu(wx.Point(ray[0]*idist, ray[1]*idist))
                    dc.DrawCirclePoint(intrs, 10)

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
            x, y, z = [int(a) for a in self.__model["CRD"]]
            self.MoveUnit(x, y, yaw, self.__model["S"])
        except KeyError : pass
        except IndexError : pass
        self.UpdateDrawing()

    def tc(self, x, y):
        x1=(x-(self.boundRect[2]+self.boundRect[0])/2)*self.__scale
        y1=(y-(self.boundRect[3]+self.boundRect[1])/2)*self.__scale
        return wx.Point(x1+self.__x0,-y1+self.__y0)

    def tsu(self, pts):
        return [self.tpu(p) for p in pts]

    def tpu(self, p):
        x, y = p.Get()
        #x1=x*self.__r_cos+y*self.__r_sin+self.__r_x+self.xu0
        #y1=-x*self.__r_sin+y*self.__r_cos+self.__r_y+self.yu0
        x1, y1 = self.UnitToMap(x, y)
        return self.tc(x1,y1)

