import json
import socket
import Queue
import threading
import math
import time

import comm
from planner import Planner
from pfilter import PFilter
from unit import Unit

import model

class Controller():
    # device controller
    def __init__(self,form,model, map, LogString, LogErrorString):
        self.__form = form
        self.__model = model
        self.__LogString=LogString
        self.__LogErrorString=LogErrorString
        self.__cmdid=0
        self.__resp_q = Queue.Queue()
        self.__lock=threading.Lock()
        self.__comm_thread = None
        self.__comm_listener_thread = None
        self.__comm_scan_thread = None
        self.__comm_sim_thread = None
        self.__comm_path_thread = None
        scan_max_dist=400
        self.planner=Planner(map, LogString, LogErrorString)
        self.pfilter=PFilter(map, scan_max_dist)
        self.unit=Unit(map, self.pfilter, scan_max_dist)

        self.__tstart()

    def __tstart(self):
        try:
            self.__comm_thread=comm.CommandThread(self, self.__model["DEVADDR"], self.__model["DEVPORT"])
            self.__comm_thread.start()
            if self.__model["SYSLOGENABLE"] :
                self.__comm_listener_thread=comm.ListenerThread(self, self.__model["LISTENPORT"])
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
        self.__comm_scan_thread=comm.ScanThread(self)
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
        self.__req({"C":"D", "RPS":[round(l,2), round(r,2)]})

    def reqMoveSpeed(self, s):
        return self.__req({"C":"M", "V":round(s,2)})

    def reqMoveSync(self, l, r):
        return self.__req_sync({"C":"D", "RPS":[round(l,2), round(r,2)]})

    def reqSteer(self, s):
        return self.__req({"C":"S", "S":round(s,2)})

    def reqSteerSync(self, s):
        return self.__req_sync({"C":"S", "S":round(s,2)})

    def reqMoveSpeedSync(self, s):
        return self.__req_sync({"C":"M", "V":round(s,2)})

    def reqBearingSync(self, s):
        return self.__req_sync({"C":"B", "A":round(s,2)})

    def reqBearing(self, s):
        return self.__req({"C": "B", "A": round(s, 2)})

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
        self.__form.ActionShow(action=js)

    def __req_sync(self, js, retries=3, timeout_net=0.25):
        js["I"]=self.__genId()
        resp_json=None

        while resp_json is None and retries>0 :
            #clean response queue first
            try:
                while True:
                    self.__resp_q.get_nowait()
            except Queue.Empty: pass
            self.__comm_thread.put((js, self.__resp_q))
            self.__form.ActionShow(action=js)
            try:
                resp = self.__resp_q.get(timeout=timeout_net)
                if resp is not None:
                    resp_json=json.loads(resp)
                    self.__form.LogString("SYNC RSP: "+resp, 'FOREST GREEN')
                    self.onResp(resp_json)
            except Queue.Empty:
                self.__form.LogErrorString("SYNC RSP MISSING")
            retries-=1
            if resp_json is None: time.sleep(0.1)
        return resp_json

    def resp(self, js, req_json=None):
        " resp callback, called in thread context!!! "
        try:
            resp_json = json.loads(js)
            if req_json is None : # syslog
                if resp_json["C"]=="A" :
                    ##self.__form.LogString("ALR: "+js, 'RED') #alarm
                    #self.__form.LogErrorString('ALR: '+js) #alarm
                    self.__form.LogErrorString(self.decodeEvent(resp_json)) #alarm
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
            #model_reset = True #this cause too much probs

        if self.__model.update(resp_json, model_reset) :
            try:
                yaw, pitch, roll = [a*math.pi/180.0 for a in self.__model["YPR"]]
                x, y, z = [int(a) for a in self.__model["CRD"]] # for simulation
                self.unit.MoveUnit(yaw, self.__model["D"], self.__model["S"], self.__model["V"], x, y)
            except KeyError : pass
            except IndexError : pass
            self.__form.UpdatePos(reset=model_reset)
        else :
            self.__form.UpdateStatus(reset=model_reset)

    def scanComplete(self, result=None):
        if result is None : self.__form.LogErrorString("Device not found")
        else : self.__form.LogString("FOUND %s" % result, 'FOREST GREEN')

    def stopPathRunning(self):
        if self.__comm_path_thread!=None:
            self.__comm_path_thread.stop()
            #time.sleep(20)
            #print("stopPathRunning-3")
            #self.__comm_path_thread.join()
            self.__comm_path_thread = None

    def isPathRunning(self):
        return self.__comm_path_thread!=None and self.__comm_path_thread.complete!=True

    def startPathRunning(self, speed):
        self.__comm_path_thread=comm.PathThread(self, self.planner, self.unit, speed)
        self.__comm_path_thread.start()

    def movePathRunning(self):
        pass

    def decodeEvent(self, js):
        mods = ['SYS', 'IMU', 'CTL']
        DELIM = ' : '
        codes=[
            [], #sys
            [(0,"NONE"), (1,"FAIL_INIT"), (2,"CVTTMO"),
             (3, "NODATA"), (4, "FIFO_OVF"), (5, "FIFO_TMO"), (6, "FIFO_EXCS"),
             (128, "INITOK")], #mpu
            [(0,'NONE'), (1,'FAIL_INIT'), (2, 'FAIL_WRT'), (3, 'FAIL_RD'), (4, 'OVF'), (5, 'ALR'), (6, 'OBST'),  (7, 'SENS_FAIL'),
             (100, 'LOG_PID'), (101, 'LOG_POW'), (102, 'LOG_PBPID0')]
        ]

        s=''
        if "C" in js and js["C"]=="A" : s+='ALR'
        else : s+='EVT'
        m=-1
        if "M" in js and js["M"]>=0 and js["M"]<len(mods) :
            m=js["M"]
            s+=DELIM+mods[m]
        else : s+=DELIM+str(m)

        if "F" in js :
            f=js["F"]
            fdecoded=None
            if m<>-1:
                codemap=codes[m]
                for cm in codemap:
                    if cm[0]==f :
                        fdecoded=cm[1]
                        break
            if fdecoded is not None:
                s+=DELIM+fdecoded
            else :
                s+=DELIM+str(f)

        if "S" in js :
            s+=DELIM+str(js["S"])

        if "P" in js :
            s+=DELIM+str(js["P"])

        return s
