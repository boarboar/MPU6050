import wx
import vlc
import sys
import wx.lib.newevent

CameraEvent, EVT_CAMERA_EVENT = wx.lib.newevent.NewEvent()

class CameraPanel(wx.Window):
    " camera panel"
    def __init__(self, parent):
        wx.Window.__init__(self, parent, wx.ID_ANY, style=wx.SIMPLE_BORDER, size=(160,120))
        self.videopanel = wx.Panel(self, -1)
        self.videopanel.SetBackgroundColour(wx.BLACK)
        sizer = wx.BoxSizer(wx.VERTICAL)
        sizer.Add(self.videopanel, 1, flag=wx.EXPAND)
        self.SetSizer(sizer)
        self.timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.OnTimer, self.timer)
        self.videopanel.Bind(wx.EVT_LEFT_UP, self.OnMouseLeftUp)
        self.Bind(EVT_CAMERA_EVENT, self.OnCameraEvent)
        self.isPlaying = False

        # VLC player controls
        try :
            self.Instance = vlc.Instance('--live-caching=0 --network-caching=0 --postproc-q=0'
                                         ' --mjpeg-fps=0 --image-realtime --no-audio ')

            #self.Instance = vlc.Instance('--live-caching=0 --network-caching=0 --postproc-q=0 --mjpeg-fps=0 '
            #                             '--image-realtime --no-audio --quiet-synchro --clock-synchro=0 --clock-jitter=0')

            self.player = self.Instance.media_player_new()
            self.event_manager = self.player.event_manager()
            self.event_manager.event_attach(vlc.EventType.MediaPlayerEncounteredError, self.http_error)
            #self.timer.Start(5000)

            self.Media = self.Instance.media_new(unicode('http://192.168.1.120:8080/?action=stream'))

            # set the window id where to render VLC's video output
            handle = self.videopanel.GetHandle()
            if sys.platform.startswith('linux'): # for Linux using the X Server
                self.player.set_xwindow(handle)
            elif sys.platform == "win32": # for Windows
                self.player.set_hwnd(handle)
            elif sys.platform == "darwin": # for MacOS
                self.player.set_nsobject(handle)

        except:
            print(sys.exc_info()[0])


    @vlc.callbackmethod
    def http_error(self, event):
        #print('VLC ERR')
        event = CameraEvent(msg='error')
        wx.PostEvent(self, event)
        return 0

    def OnCameraEvent(self, evt):
        self.timer.Start(5000)

    def OnTimer(self, evt):
        self.timer.Stop()
        if not self.isPlaying : return
        self.player.set_media(self.Media)
        if self.player.play() == -1:
                print("Unable to play.")

    def OnMouseLeftUp(self, evt):
        if self.isPlaying :
            self.timer.Stop()
            self.isPlaying=False
            self.player.stop()
        else :
            self.isPlaying=True
            self.timer.Start(1000)
