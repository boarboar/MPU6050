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
        self.Bind(wx.EVT_MOUSEWHEEL, self.OnMouseWheel)
        self.Bind(wx.EVT_LEFT_UP, self.OnMouseLeftUp)
        self.Bind(wx.EVT_LEFT_DOWN, self.OnMouseLeftDown)
        self.Bind(wx.EVT_MOTION, self.OnMouseMotion)
        self.__model=model
        self.__map=[]
        self.__x0, self.__y0 = (0, 0) # canvas center
        self.__scale=1
        self.__drag=False
        self.__dragged=False
        self.__drag_prev=(0,0)
        self.__drag_delta=(0, 0)
        self.__pos_set_on=False
        self.__shape=[wx.Point(-self.UNIT_WIDTH/2, -self.UNIT_HEIGHT/2),
                    wx.Point(-self.UNIT_WIDTH/2, self.UNIT_HEIGHT/2),
                    wx.Point(0, self.UNIT_HEIGHT*3/5),
                    wx.Point(self.UNIT_WIDTH/2, self.UNIT_HEIGHT/2),
                    wx.Point(self.UNIT_WIDTH/2, -self.UNIT_HEIGHT/2),
                    ]
        #self.InitParticles()
        self.OnSize(None)

    def OnSize(self,event):
        Size  = self.ClientSize
        self._Buffer = wx.EmptyBitmap(*Size)
        self.InitPosition()
        #self.__initsz = True
        self.UpdateDrawing()

    def OnPaint(self, event):
        dc = wx.BufferedPaintDC(self, self._Buffer)

    def OnMouseWheel(self,event):
        scale_factor = 1.1
        if event.GetWheelRotation() < 0 : scale_factor=1/1.1
        self.Scale(scale_factor)

    def OnMouseLeftDown(self,event):
        self.__drag=True
        self.__drag_prev=self.ScreenToClient(wx.GetMousePosition())
        #X, Y= self.ScreenToClient(wx.GetMousePosition())
        #print(X,Y)
        #self.__drag_delta=(self.__x0-X, self.__x0-Y)
        event.Skip()

    def OnMouseLeftUp(self,event):
        self.__drag=False
        if not self.__dragged and self.__pos_set_on:
            print "Click"
            X, Y = self.ScreenToClient(wx.GetMousePosition())
            self.xu0, self.yu0 = self.tm(X, Y)
            self.UpdateDrawing()
        self.__dragged=False
        event.Skip()

    def OnMouseMotion(self,event):
        if self.__drag :
            X, Y = self.ScreenToClient(wx.GetMousePosition())
            #self.__x0 = self.__drag_delta[0]+X
            #self.__y0 = self.__drag_delta[1]+Y
            self.__x0 += X-self.__drag_prev[0]
            self.__y0 += Y-self.__drag_prev[1]
            self.__drag_prev = (X,Y)
            self.__dragged=True
            self.UpdateDrawing()

    def onFit(self,event):
        self.InitPosition()
        self.UpdateDrawing()

    def onZoom(self,event,dir):
        scale={"in":1.2,"out":1.0/1.2}
        print('inzoom1')
        self.Scale(scale[dir])
        print('inzoom2')

    def onButtonMove(self,event,dir):
        move={"left":(40, 0),"right":(-40, 0),"up":(0, 40), "dn":(0, -40)}
        offs=move[dir]
        self.__x0+=offs[0]
        self.__y0+=offs[1]
        print("(%s, %s) @ %s" % (self.__x0, self.__y0, self.__scale))
        self.UpdateDrawing()

    def onPosToggle(self,event,state):
        self.__pos_set_on=state

    def InitPosition(self):
        Size  = self.ClientSize
        self.__x0=Size.width/2
        self.__y0=Size.height/2
        w = (self.boundRect[2]-self.boundRect[0])*1.1
        h = (self.boundRect[3]-self.boundRect[1])*1.1
        if w>0 :
            self.__scale=Size.width/w
        if h>0 and Size.height/h<self.__scale:
            self.__scale=Size.height/h

    def Scale(self, scale_factor):
        self.__scale *=scale_factor
        Size  = self.ClientSize
        self.__x0 = Size.width/2+(self.__x0-Size.width/2)*scale_factor
        self.__y0 = Size.height/2+(self.__y0-Size.height/2)*scale_factor
        print("(%s, %s) @ %s" % (self.__x0, self.__y0, self.__scale))
        self.UpdateDrawing()

    def Draw(self, dc):
        dc.Clear()
        self.DrawAreas(dc)
        self.DrawParticles(dc)
        self.DrawRobot(dc)

    def DrawAreas(self, dc):
        dc.SetTextForeground(wx.BLACK)
        dc.SetTextBackground(wx.WHITE)
        dc.SetBackgroundMode(wx.SOLID)
        #dc.SetBrush(wx.Brush("GREY"))

        obj_brush_hi_dens=wx.Brush("GREY")
        obj_brush_lo_dens=wx.Brush("LIGHT GREY")
        wall_pen=wx.Pen(wx.BLACK, 4)
        obj_pen=wx.Pen(wx.BLACK, 2)
        obj_pen_hidden=wx.Pen(wx.BLACK, 2, wx.PENSTYLE_SHORT_DASH)
        door_pen_open=wx.Pen("GREEN", 4)
        door_pen_closed=wx.Pen("RED", 4)
        door_pen_undef=wx.Pen("YELLOW", 4)

        try:
            for area in self.map["AREAS"] :
                parea0=area["AT"]
                for wall in area["WALLS"] :
                    pen=wall_pen
                    if wall["T"]=="D" : # DOOR
                        status=0 #closed
                        if 'S' in wall : status=wall["S"]
                        if status==0 : pen=door_pen_closed
                        elif status==1 : pen=door_pen_open
                        else : pen=door_pen_undef
                    dc.SetPen(pen)
                    dc.DrawLinePoint(self.tc(parea0[0]+wall["C"][0],parea0[1]+wall["C"][1]),
                                     self.tc(parea0[0]+wall["C"][2],parea0[1]+wall["C"][3]))
                # optional - objects
                try :
                    for obj in area["OBJECTS"] :
                        brush=obj_brush_hi_dens
                        pen = obj_pen
                        if obj["DENSITY"] <= 0.5 : brush=obj_brush_lo_dens
                        if obj["DENSITY"] < 0.1 : pen=obj_pen_hidden
                        dc.SetBrush(brush)
                        dc.SetPen(pen)
                        pobj0=(parea0[0]+obj["AT"][0],
                               parea0[1]+obj["AT"][1])
                        pts=[]
                        for c in obj["CS"] :
                            p = c["C"]
                            pts.append(self.tc(pobj0[0]+p[0],
                                               pobj0[1]+p[1]))
                        dc.DrawPolygon(pts)

                except KeyError : pass
                except IndexError : pass
        except KeyError : pass
        except IndexError : pass

        dc.SetBackgroundMode(wx.TRANSPARENT)
        dc.SetBrush(wx.TRANSPARENT_BRUSH)

        zero = self.tc(self.xu0, self.yu0)
        dc.SetPen(wx.Pen(wx.BLACK, 1))
        dc.DrawLine(zero.x-10, zero.y, zero.x+10, zero.y)
        dc.DrawLine(zero.x, zero.y-10, zero.x, zero.y+10)
        zero = self.tc(0, 0)
        dc.DrawTextPoint("(0,0)", zero)

    def DrawParticles(self, dc):
        dc.SetBackgroundMode(wx.SOLID)
        dc.SetBrush(wx.RED_BRUSH)

        c_pen=wx.Pen(wx.RED, 1)
        c_pen_0=wx.Pen("GRAY", 1)
        p_ray_pen=wx.Pen("GRAY", 1, wx.PENSTYLE_SHORT_DASH)

        for p in self.particles :
            rad=1+math.log10(1+p.w*10)*8
            c=self.tc(p.x,p.y)
            if rad>1 : dc.SetPen(c_pen)
            else : dc.SetPen(c_pen_0)
            dc.DrawCirclePoint(c, rad)
            ca=wx.Point(c.x+10*math.sin(p.a), c.y-10*math.cos(p.a))
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

        # draw estimation and variance
        dc.SetBackgroundMode(wx.TRANSPARENT)
        dc.SetBrush(wx.TRANSPARENT_BRUSH)
        #ray_pen=wx.Pen(wx.BLACK, 1, wx.PENSTYLE_LONG_DASH)
        mx, my, var, ma, vara = self.getMeanDistribution()
        c=self.tc(mx,my)
        dc.SetPen(wx.Pen(wx.BLACK, 2))
        dc.DrawCirclePoint(c, var*self.__scale)
        c1a=wx.Point(c.x+30*math.sin(ma-vara), c.y-30*math.cos(ma-vara))
        c2a=wx.Point(c.x+40*math.sin(ma), c.y-40*math.cos(ma))
        c3a=wx.Point(c.x+30*math.sin(ma+vara), c.y-30*math.cos(ma+vara))
        dc.DrawPolygon([c, c1a, c2a, c3a])

    def DrawRobot(self, dc):
        dc.SetBackgroundMode(wx.TRANSPARENT)
        dc.SetBrush(wx.TRANSPARENT_BRUSH)
        ray_pen=wx.Pen(wx.BLACK, 1, wx.PENSTYLE_LONG_DASH)
        ray_pen_ref=wx.Pen('GREY', 1, wx.DOT)
        if self.isInside :
            dc.SetPen(wx.Pen(wx.BLUE, 2))
        else :
            dc.SetPen(wx.Pen(wx.RED, 2))

        dc.DrawPolygon(self.tsu(self.__shape))
        # draw sensor rays
        if self.isInside :
            i=0
            inters_pen=wx.Pen(wx.GREEN, 2)
            inters_pen_c=wx.Pen(wx.BLUE, 2)
            inters_pen_cr=wx.Pen('GRAY', 2)
            for ray in self.scan_rays:
                dc.SetPen(ray_pen)
                dc.DrawLinePoint(self.tpu(wx.Point(0, 0)), self.tpu(wx.Point(ray[0]*self.scan_max_dist, ray[1]*self.scan_max_dist)))
                # draw measured intersection
                idist=self.scans[i]
                i=i+1
                if idist>0 :
                    dc.SetPen(inters_pen)
                    intrs = self.tpu(wx.Point(ray[0]*idist, ray[1]*idist))
                    dc.DrawCirclePoint(intrs, 10)
                # draw calculate intersections


                #intrs0, pr, intrs1, refstate, intrs=self.getIntersectionMapRefl(self.UnitToMap(0, 0),
                #                                   self.UnitToMap(ray[0]*self.scan_max_dist, ray[1]*self.scan_max_dist))

                intrs0, pr, intrs1, refstate, intrs=self.getIntersectionUnit(0,0,ray[0]*self.scan_max_dist, ray[1]*self.scan_max_dist)
                if intrs0 != None :
                    if pr != None :
                        dc.SetPen(ray_pen_ref)
                        dc.DrawLinePoint(self.tc(intrs0[0], intrs0[1]), self.tc(pr[0], pr[1]))
                    if refstate :
                        dc.SetPen(inters_pen_cr)
                        dc.DrawCirclePoint(self.tc(intrs0[0], intrs0[1]), 5)
                if intrs!=None :
                    dc.SetPen(inters_pen_c)
                    dc.DrawCirclePoint(self.tc(intrs[0], intrs[1]), 5)


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

        Size  = self.ClientSize
        pos=self.tpu(wx.Point(0, 0))
        if pos.x < 0 : self.__x0+=-pos.x+20
        if pos.y < 0 : self.__y0+=-pos.y+20
        if pos.x > Size.width : self.__x0+=(Size.width-pos.x-20)
        if pos.y > Size.height : self.__y0+=(Size.height-pos.y-20)
        self.UpdateDrawing()

    def tc(self, x, y):
        # Map to screen
        x1=(x-(self.boundRect[2]+self.boundRect[0])/2)*self.__scale
        y1=(y-(self.boundRect[3]+self.boundRect[1])/2)*self.__scale
        return wx.Point(x1+self.__x0,-y1+self.__y0)

    def tsu(self, pts):
        return [self.tpu(p) for p in pts]

    def tpu(self, p):
        # Unit to screen
        x, y = p.Get()
        x1, y1 = self.UnitToMap(x, y)
        return self.tc(x1,y1)

    def tm(self, x, y):
        # Screen to Map
        x=x-self.__x0
        y=-y+self.__y0
        x=x/self.__scale+(self.boundRect[2]+self.boundRect[0])/2
        y=y/self.__scale+(self.boundRect[3]+self.boundRect[1])/2
        return (x,y)
