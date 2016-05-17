import json
import socket
import Queue
import threading
import comm
import model

class Controller():
    # device controller
    def __init__(self,form,model):
        self.__form = form
        self.__model = model
        self.__cmdid=0
        self.__resp_q = Queue.Queue()
        self.__lock=threading.Lock()
        self.__comm_thread = None
        self.__comm_listener_thread = None
        self.__comm_scan_thread = None
        self.__comm_sim_thread = None
        self.__comm_path_thread = None
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

    def stop(self, timeout=None):
        self.__form.LogString("Stopping...")
        if self.__comm_listener_thread is not None :
            self.__comm_listener_thread.stop()
            self.__comm_listener_thread.join()
        self.__comm_thread.stop()
        self.__comm_thread.join()
        self.stopScan()
        self.stopSimulation()
        self.__form.LogString("Stopped")

    def restart(self):
        self.stop()
        self.__tstart()

    def startScan(self):
        self.__comm_scan_thread=comm.ScanThread(self, self.__model["MOCKUP"])
        self.__comm_scan_thread.start()

    def stopScan(self):
        if self.__comm_scan_thread!=None:
            self.__comm_scan_thread.stop()
            self.__comm_scan_thread.join()
            self.__comm_scan_thread = None

    def isScanning(self):
        return self.__comm_scan_thread!=None

    def startSimulation(self):
        self.__comm_sim_thread=comm.SimulationThread(self)
        self.__comm_sim_thread.start()

    def stopSimulation(self):
        if self.__comm_sim_thread!=None:
            self.__comm_sim_thread.stop()
            self.__comm_sim_thread.join()
            self.__comm_sim_thread = None

    def isSimulating(self):
        return self.__comm_sim_thread!=None

    def reqCmdRaw(self, cmd):
        try:
            js=json.loads(cmd)
            self.__req(js)
        except ValueError: pass

    def reqStatus(self):
        # {"I":1,"C":"INFO"}
        self.__req({"C": "INFO"})

    def reqResetMPU(self, action="MPU", pos=(0,0)):
        # {"I":1,"C":"RSTMPU", "A": action, "P": [x, y]}
        self.__req({"C": "RSTMPU", "A": action, "P": pos})

    def reqPosition(self):
        # {"I":1,"C":"POS"}
        self.__req({"C": "POS"})

    def reqPositionSync(self):
        # {"I":1,"C":"POS"}
        return self.__req_sync({"C": "POS"})

    def reqMove(self, l, r):
        self.__req({"C":"D", "RPS":[l, r]})

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
        js["I"]=self.__genId()
        self.__comm_thread.put((js, None))

    def __req_sync(self, js):
        js["I"]=self.__genId()
        resp_json=None
        self.__comm_thread.put((js, self.__resp_q))
        try:
            resp = self.__resp_q.get(timeout=1)
            resp_json=json.loads(resp)
            self.__form.LogString("SYNC RSP: "+resp, 'FOREST GREEN')
            self.onResp(resp_json)
        except Queue.Empty: resp_json = None
        return resp_json

    def resp(self, js, req_json=None):
        " resp callback, called in thread context!!! "
        try:
            resp_json = json.loads(js)
            if req_json is None : # syslog
                if resp_json["C"]=="A" :
                    self.__form.LogString("ALR: "+js, 'RED') #alarm
                else :
                    self.__form.LogString("LOG: "+js, 'BLUE') #event
            #elif req_json is not None : #cmd-rsp
            #    if int(req_json["I"]) != int(resp_json["I"]) :
            #        self.__form.LogErrorString("UNMATCHED: "+js)
            #        return False
            else:
                self.__form.LogString("RSP: "+js, 'FOREST GREEN')
        except ValueError : return True
        except KeyError : return True

        self.onResp(resp_json)

        return True

    def __genId(self):
        self.__lock.acquire()
        self.__cmdid=self.__cmdid+1
        cmdid=self.__cmdid
        self.__lock.release()
        return cmdid

    def onResp(self, resp_json):
        model_reset=False
        #if "I" in resp_json and resp_json["T"] < self.__model["T"] : # command resp has time less saved one...
        if "T" in resp_json and resp_json["T"] < self.__model["T"] : # command resp has time less saved one...
            self.__form.LogErrorString("Device time is less than old one, Device is likely to reboot")
            model_reset = True

        if self.__model.update(resp_json, model_reset) :
            self.__form.UpdatePos(reset=model_reset)
        else :
            self.__form.UpdateStatus(reset=model_reset)

    def scanComplete(self, result=None):
        if result is None : self.__form.LogErrorString("Device not found")
        else : self.__form.LogString("FOUND %s" % result, 'FOREST GREEN')

    def stopPathRunning(self):
        if self.__comm_path_thread!=None:
            self.__comm_path_thread.stop()
            self.__comm_path_thread.join()
            self.__comm_path_thread = None

    def isPathRunning(self):
        return self.__comm_path_thread!=None

    def startPathRunning(self, planner, unit):
        self.__comm_path_thread=comm.PathThread(self, planner, unit)
        self.__comm_path_thread.start()

