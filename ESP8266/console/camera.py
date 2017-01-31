import wx
import wx.html2

class CameraPanel(wx.Window):
    " camera panel"
    def __init__(self, parent):
        wx.Window.__init__(self, parent, wx.ID_ANY, style=wx.SIMPLE_BORDER, size=(160,160))
        self.SetBackgroundColour('BLACK')
        sizer = wx.BoxSizer(wx.VERTICAL)
        self.browser = wx.html2.WebView.New(self)
        sizer.Add(self.browser, 1, wx.EXPAND, 10)
        self.SetSizer(sizer)
        #self.browser.LoadURL("http://www.google.com")
        #self.browser.LoadURL("http://88.53.197.250/axis-cgi/mjpg/video.cgi?resolution=320x240")
        #self.browser.LoadURL("http://192.168.1.120:8080/")
        self.browser.LoadURL("http://192.168.1.120:8080/?action=stream")
