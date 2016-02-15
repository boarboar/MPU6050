import comm
import json
import socket
import model

class Controller():
    # device controller
    def __init__(self,form,model):
        self.__form = form
        self.__model = model
        self.__cmdid=0
        self.__comm_scan_thread = None
        self.__tstart()

    def __tstart(self):
        try:
            self.__comm_thread=comm.CommandThread(self, self.__model["DEVADDR"], self.__model["DEVPORT"], self.__model["MOCKUP"])
            self.__comm_thread.start()
            if self.__model["SYSLOGENABLE"] :
                self.__comm_listener_thread=comm.ListenerThread(self, self.__model["LISTENPORT"], self.__model["MOCKUP"])
                self.__comm_listener_thread.start()
            else :
                self.__comm_listener_thread=None
        except KeyError: pass

    def log(self): return self.__form

    def restart(self):
        self.__form.LogString("Stopping...")
        if self.__comm_listener_thread is not None :
            self.__comm_listener_thread.stop()
            self.__comm_listener_thread.join()
        self.__comm_thread.stop()
        self.__comm_thread.join()
        self.__form.LogString("Restarting...")
        self.__tstart()

    def startScan(self):
        self.__comm_scan_thread=comm.ScanThread(self, self.__model["MOCKUP"])
        self.__comm_scan_thread.start()

    def stopScan(self):
        self.__comm_scan_thread.stop()
        self.__comm_scan_thread.join()
        self.__comm_scan_thread = None

    def isScanning(self):
        return self.__comm_scan_thread!=None
        #if self.__comm_scan_thread==None : return False
        #return True

    def reqCmdRaw(self, cmd):
        try:
            js=json.loads(cmd)
            self.__req(js)
        except ValueError: pass

    def reqStatus(self):
        # {"I":1,"C":"INFO"}
        self.__req({"C": "INFO"})

    def reqPosition(self):
        # {"I":1,"C":"POS"}
        self.__req({"C": "POS"})

    def reqUpload(self):
        # config upload
        # {"I":1,"C":"SYSL", "ON":1, "ADDR":"192.168.1.141", "PORT":4444}
        js={"C": "SYSL"}
        try:
            js["ON"]=int(self.__model["SYSLOGENABLE"])
            js["ADDR"]=str(socket.gethostbyname(socket.gethostname()))
            js["PORT"]=int(self.__model["LISTENPORT"])
            self.__req(js)
        except KeyError : pass

    def __req(self, js):
        self.__cmdid=self.__cmdid+1
        js["I"]=self.__cmdid
        self.__comm_thread.put(js)

    def resp(self, js, req_json=None):
        " resp callback, called in thread context!!! "
        if req_json is None : self.__form.LogString("SYS:"+js, 'BLUE') #syslog
        try:
            resp_json = json.loads(js)
            if req_json is not None : #cmd-rsp
                if int(req_json["I"]) != int(resp_json["I"]) :
                    self.__form.LogErrorString("UNMATCHED: "+js)
                    return False
                self.__form.LogString("RSP:"+js, 'FOREST GREEN')
        except ValueError : return True
        except KeyError : return True

        self.__model.update(resp_json)
        #if resp_json["C"]=="POS" :
        self.__form.UpdatePos()
        return True

    def scanComplete(self, result=None):
        if result is None :
            self.__form.LogErrorString("Device not found")
        else : self.__form.LogString("FOUND %s" % result, 'FOREST GREEN')
