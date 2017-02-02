import wx
import wx.html2

class CameraPanel(wx.Window):
    " camera panel"
    def __init__(self, parent):
        wx.Window.__init__(self, parent, wx.ID_ANY, style=wx.SIMPLE_BORDER, size=(320,240))
        self.SetBackgroundColour('BLACK')
        sizer = wx.BoxSizer(wx.VERTICAL)
        self.browser = wx.html2.WebView.New(self)
        sizer.Add(self.browser, 1, wx.EXPAND, 10)
        self.SetSizer(sizer)
        #self.browser.LoadURL("http://www.google.com")
        #self.browser.LoadURL("http://192.168.1.120:8080/")
        self.browser.LoadURL("http:root@tombaz@//192.168.1.120/")


        #self.browser.LoadURL("http://192.168.1.1/")
        self.Bind(wx.html2.EVT_WEBVIEW_LOADED, self.OnPageLoaded, self.browser)

    def OnPageLoaded(self, evt):
        print("loaded")
        self.Bind(wx.html2.EVT_WEBVIEW_LOADED, None, self.browser)
