import wx
import vlc
import sys

class CameraPanel(wx.Window):
    " camera panel"
    def __init__(self, parent):
        wx.Window.__init__(self, parent, wx.ID_ANY, style=wx.SIMPLE_BORDER, size=(160,120))
        self.videopanel = wx.Panel(self, -1)
        self.videopanel.SetBackgroundColour(wx.BLACK)
        sizer = wx.BoxSizer(wx.VERTICAL)
        sizer.Add(self.videopanel, 1, flag=wx.EXPAND)
        self.SetSizer(sizer)
        # VLC player controls
        try :
            self.Instance = vlc.Instance('--live-caching=10 --network-caching=10 --http-reconnect --http-continuous --repeat')

            #--live-caching=

            self.player = self.Instance.media_player_new()
            self.Media = self.Instance.media_new(unicode('http://192.168.1.120:8080/?action=stream'))
            self.player.set_media(self.Media)
            # set the window id where to render VLC's video output
            handle = self.videopanel.GetHandle()
            if sys.platform.startswith('linux'): # for Linux using the X Server
                self.player.set_xwindow(handle)
            elif sys.platform == "win32": # for Windows
                self.player.set_hwnd(handle)
            elif sys.platform == "darwin": # for MacOS
                self.player.set_nsobject(handle)

            #self.event_manager = self.Media.event_manager()
            #self.event_manager = vlc.libvlc_media_player_event_manager(self)
            self.event_manager = self.player.event_manager()
            self.event_manager.event_attach(vlc.EventType.MediaPlayerEncounteredError, self.http_error)

            if self.player.play() == -1:
                print("Unable to play.")

        except:
            print(sys.exc_info()[0])


    @vlc.callbackmethod
    def http_error(self, event):
        print('VLC ERR')
        return 0