import wx
import json
from pprint import pprint

class MapPanel(wx.Window):
    " draw panel"
    def __init__(self, parent, mapfile):
        wx.Window.__init__(self, parent, wx.ID_ANY, style=wx.SIMPLE_BORDER, size=(240,240))
        self.Bind(wx.EVT_PAINT, self.OnPaint)
        self.Bind(wx.EVT_SIZE, self.OnSize)
        self.__map=[]
        try:
            with open(mapfile) as data_file:
                self.__map = json.load(data_file)
            #pprint(__map)
        #except IOError: pass
        except : pass

    def OnSize(self,event):
        Size  = self.ClientSize
        self.x0=Size.width/2
        self.y0=Size.height/2
        self.UpdateDrawing()

    def OnPaint(self, event=None):
        dc = wx.PaintDC(self)
        dc.Clear()
        dc.SetPen(wx.Pen(wx.BLACK, 1))
        try:
            for area in self.__map["AREAS"] :
                for wall in area["WALLS"] :
                    dc.DrawLine(self.x0+wall["C"][0],self.y0-wall["C"][1],self.x0+wall["C"][2],self.y0-wall["C"][3])
        except KeyError : pass
        except IndexError : pass

    def UpdateDrawing(self) :
        self.Refresh()
        self.Update()

    def UpdateData(self, t, pos=None):
        pass