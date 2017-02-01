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
        self.browser.LoadURL("http://192.168.1.120/")
        #self.browser.LoadURL("http://intra.reksoft.ru/my")


        #self.browser.LoadURL("http://192.168.1.1/")
        self.Bind(wx.html2.EVT_WEBVIEW_LOADED, self.OnPageLoaded, self.browser)

    def OnPageLoaded(self, evt):
        print("loaded")
        self.Bind(wx.html2.EVT_WEBVIEW_LOADED, None, self.browser)

    def OnPageLoaded1(self, evt):
        self.browser.RunScript("""
            // There are probably better ways to get the elements you
            // want, but this works.
            document.getElementsByName('name')[0].value="hist";
            document.getElementsByName('password')[0].value="bar";

            document.getElementById('openididentifier').value="ident";

            // If you want to submit the form you can use something like
            //document.getElementsByName('login')[1].click()
            """)
        # And you probably want to unbind the event here
        self.Bind(wx.html2.EVT_WEB_VIEW_LOADED, None, self.browser)