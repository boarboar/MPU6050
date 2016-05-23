import threading
import Queue
import random
import json
import socket
import time
import math

class CommandThread(threading.Thread):
    # device command-resp communication
    def __init__(self, controller, addr, port, mockup=False):
        threading.Thread.__init__(self)
        self.__controller = controller
        self.__q = Queue.Queue()
        self.__addr=addr
        self.__port=int(port)
        self.__mockup=mockup
        self.__stop = False
        self.setDaemon(1)

    def put(self, msg):
        self.__q.put_nowait(msg)

    def stop(self) : self.__stop=True

    def run (self):
        try:
            self.__s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.__s.settimeout(2)
        except socket.error:
            self.__controller.log().LogErrorString("Failed to create socket!")
            return

        self.__controller.log().LogString("Cmd thread starting: dev=%s:%s" % (self.__addr, str(self.__port)))
        while not self.__stop:
            try:
                req_json, resp_q = self.__q.get(timeout=1)
            except Queue.Empty: req_json = None
            if req_json is not None:
                self.__controller.log().LogString("REQ: %s" % json.dumps(req_json))
                if self.__mockup :
                    try:
                        if req_json["C"]=="INFO" : rsp_js = json.dumps({"C": "INFO", "FHS": int(random.random()*40000), "FSS": (int)(random.random()*200000)})
                        elif req_json["C"]=="POS" : rsp_js = json.dumps({"C": "POS", "X": int(random.random()*100), "Y": int(random.random()*100)})
                        else : rsp_js = ""
                        self.__controller.resp(rsp_js)
                    except KeyError: continue
                    except ValueError: continue
                else :
                    try :
                        self.__s.sendto(json.dumps(req_json), (self.__addr, self.__port))
                        retr=0
                        while retr<3 :
                            d = self.__s.recvfrom(1024)
                            #self.__controller.log().LogString("From %s rsp %s" % (d[1], d[0]), 'GREY')
                            resp_json=json.loads(d[0])
                            if int(req_json["I"]) != int(resp_json["I"]) :
                                self.__controller.log().LogErrorString("UNMATCHED: "+d[0])
                            else:
                                if resp_q is not None:
                                    resp_q.put_nowait(d[0])
                                else:
                                    self.__controller.resp(d[0], req_json)
                                break
                    except socket.timeout as msg:
                        self.__controller.log().LogErrorString("Timeout")
                    except socket.error as msg:
                        self.__controller.log().LogErrorString("Sock error : %s" % msg)

        self.__s.close()
        self.__controller.log().LogString("Cmd thread stopped")

class ListenerThread(threading.Thread):
    # device command-resp communication
    def __init__(self, controller, bindport, mockup=False):
        threading.Thread.__init__(self)
        self.__controller = controller
        self.__bindport=bindport
        self.__mockup=mockup
        self.__stop = False
        self.setDaemon(1)
    def stop(self) : self.__stop=True
    def run (self):
        try:
            self.__s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.__s.settimeout(1)
        except socket.error:
            self.__controller.log().LogErrorString("Failed to create syslog socket!")
            return
        # Bind socket to local host and port
        try:
            self.__s.bind(("", int(self.__bindport)))
        except socket.error as msg:
            self.__controller.log().LogErrorString("Failed to bind syslog socket!")
            return

        self.__controller.log().LogString("Syslog thread starting on %s" % (str(self.__bindport)))
        while not self.__stop:
            try :
                d = self.__s.recvfrom(1024)
                #self.__controller.log().LogString("From %s log %s" % (d[1], d[0]), 'GREY')
                self.__controller.resp(d[0])
            except socket.timeout as msg:
                pass
            except socket.error as msg:
                self.__controller.log().LogErrorString("Sock error : %s" % msg)

        self.__controller.log().LogString("Syslog thread stopped")

class ScanThread(threading.Thread):
    # device command-resp communication
    def __init__(self, controller, mockup=False):
        threading.Thread.__init__(self)
        self.__controller = controller
        self.__mockup=mockup
        self.__stop = False
        self.setDaemon(1)
    def stop(self) : self.__stop=True
    def run (self):
        try:
            self.__s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.__s.settimeout(1)
        except socket.error:
            self.__controller.log().LogErrorString("Failed to create syslog socket!")
            return

        self.__controller.log().LogString("Scanner thread started")

        cmd='{"I": 1, "C": "INFO"}'
        port = 4444
        id=1
        result = None
        while not self.__stop and id<255:
            try :
                addr='192.168.1.'+str(id)
                id=id+1
                self.__controller.log().LogString("Scanning %s..." % (addr), 'GREY')
                self.__s.sendto(cmd, (addr, port))
                d = self.__s.recvfrom(1024)
                self.__controller.log().LogString("From %s rsp %s" % (d[1], d[0]), 'GREY')
                resp_json=json.loads(d[0])
                if resp_json["C"]=="INFO" and "I" in resp_json and "FHS"  in resp_json:
                    result=addr
                    break
            except socket.timeout as msg:
                pass
            except socket.error as msg:
                self.__controller.log().LogErrorString("Sock error : %s" % msg)
            except KeyError : return True

        self.__controller.scanComplete(result)

        self.__controller.log().LogString("Scanner thread stopped")

class SimulationThread(threading.Thread):
    # device command-resp communication
    def __init__(self, controller):
        threading.Thread.__init__(self)
        self.__controller = controller
        self.__stop = False
        self.setDaemon(1)
    def stop(self) : self.__stop=True
    def run (self):

        self.__controller.log().LogString("Starting simulation")

        #self.__controller.reqResetMPU()

        while not self.__stop :
            time.sleep(1)
            self.__controller.reqPosition()

        self.__controller.log().LogString("Simulation thread stopped")


class PathThread(threading.Thread):
    # device command-resp communication
    def __init__(self, controller, planner, unit):
        threading.Thread.__init__(self)
        self.__controller = controller
        self.__planner = planner
        self.__unit = unit
        self.__stop = False
        self.complete = False
        self.setDaemon(1)

    def stop(self) : self.__stop=True

    def run (self):
        self.__controller.log().LogString("Starting path running")
        move_var=[0.25, 0.25]
        move_var_lim=0.25
        base_move=0.25
        #self.__controller.reqMove(base_move+move_var[0],base_move+move_var[1])
        gain_p, gain_d, gain_i, gain_f = 0.5, 2.0, 0.01, 0.05
        degain_i=0.5
        err_p_0=0
        err_i=0
        self.complete=False

        while not self.__stop:  #and not within target ....
            resp_json = self.__controller.reqPositionSync()
            try:
                if resp_json is not None and resp_json["C"]=="POS":
                    time.sleep(0.5)
                    self.__planner.RePlanOnMove((self.__unit.x_mean,self.__unit.y_mean), False)

                    if len(self.__planner.spath)<2 :
                        self.__controller.log().LogString("Got there!")
                        self.complete=True
                        break

                    plan_a=math.atan2( self.__planner.spath[1][0]-self.__planner.spath[0][0],
                                        self.__planner.spath[1][1]-self.__planner.spath[0][1])

                    self.__controller.log().LogString("After move CRD=(%s %s), A=%s, AP=%s"
                                                      % (round(self.__unit.x_mean,2), round(self.__unit.y_mean, 2),
                                                         round(self.__unit.a_mean*180/math.pi,2),
                                                      round(plan_a*180/math.pi,2)))
                    err_p=(self.__unit.a_mean-plan_a)
                    if err_p < -math.pi :
                        err_p += 2*math.pi
                    if err_p > math.pi :
                        err_p -= 2*math.pi

                    #err_p=(self.__unit.a_mean-plan_a)%math.pi   # mod 180deg
                    err_i=err_p+degain_i*err_i
                    err_d=err_p-err_p_0
                    err_p_0=err_p
                    feedback=-(gain_p*err_p+gain_d*err_d+gain_i*err_i)*gain_f

                    self.__controller.log().LogString("PID: %s %s %s => %s" %
                                                      (round(err_p,2), round(err_d,2), round(err_i,2), round(feedback,2)))

                    dmove=[feedback, -feedback]
                    for im in range(2):
                        move_var[im]+=dmove[im]
                        if move_var[im]>move_var_lim: move_var[im]=move_var_lim
                        if move_var[im]<-move_var_lim: move_var[im]=-move_var_lim

                    self.__controller.reqMove(base_move+move_var[0], base_move+move_var[1])

            except KeyError: pass

            time.sleep(2)

        self.__controller.reqMove(0,0)
        self.__controller.log().LogString("Path running thread stopped")