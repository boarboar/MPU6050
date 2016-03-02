import wx
import json
import sys
from pprint import pprint

class MapPanel(wx.Window):
    " draw panel"
    def __init__(self, parent, mapfile):
        wx.Window.__init__(self, parent, wx.ID_ANY, style=wx.SIMPLE_BORDER, size=(240,240))
        self.Bind(wx.EVT_PAINT, self.OnPaint)
        self.Bind(wx.EVT_SIZE, self.OnSize)
        self.__map=[]
        self.__x0=0
        self.__y0=0
        self.__scale=1
        self.__boundRect=[sys.maxint, sys.maxint, -sys.maxint, -sys.maxint]
        try:
            with open(mapfile) as data_file:
                self.__map = json.load(data_file)
            #pprint(__map)
            for area in self.__map["AREAS"] :
                for wall in area["WALLS"] :
                    self.adjustBound(wall["C"][0], wall["C"][1])
                    self.adjustBound(wall["C"][2], wall["C"][3])
            print("Map loaded")
        #except IOError: pass
        except : pass

    def adjustBound(self, x, y):
        if x<self.__boundRect[0] : self.__boundRect[0]=x
        if y<self.__boundRect[1] : self.__boundRect[1]=y
        if x>self.__boundRect[2] : self.__boundRect[2]=x
        if y>self.__boundRect[3] : self.__boundRect[3]=y

    def tc(self, x, y):
        x1=(x-(self.__boundRect[2]-self.__boundRect[0])/2)*self.__scale
        y1=(y-(self.__boundRect[3]-self.__boundRect[1])/2)*self.__scale
        return wx.Point(x1+self.__x0,-y1+self.__y0)

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

        self.UpdateDrawing()

    def OnPaint(self, event=None):
        dc = wx.PaintDC(self)
        dc.Clear()
        wall_pen=wx.Pen(wx.BLACK, 4)
        door_pen=wx.Pen("GRAY", 4)

        try:
            for area in self.__map["AREAS"] :
                for wall in area["WALLS"] :
                    if wall["T"]=="W" :
                        dc.SetPen(wall_pen)
                    else :
                        dc.SetPen(door_pen)
                    dc.DrawLinePoint(self.tc(wall["C"][0],wall["C"][1]),self.tc(wall["C"][2],wall["C"][3]))
        except KeyError : pass
        except IndexError : pass

    def UpdateDrawing(self) :
        self.Refresh()
        self.Update()

    def UpdateData(self, t, pos=None):
        pass