import threading
import Queue
import random
import json
import socket
import time
import math

class CommandThread(threading.Thread):
    # device command-resp communication
    NRET=3
    def __init__(self, controller, addr, port):
        threading.Thread.__init__(self)
        self.__controller = controller
        self.__q = Queue.Queue()
        self.__addr=addr
        self.__port=int(port)
        self.__stop = False
        self.setDaemon(1)

    def put(self, msg):
        self.__q.put_nowait(msg)

    def stop(self) : self.__stop=True

    def run (self):
        try:
            self.__s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.__s.settimeout(0.5)
        except socket.error:
            self.__controller.log().LogErrorString("Failed to create socket!")
            return

        self.__controller.log().LogString("Cmd thread starting: dev=%s:%s" % (self.__addr, str(self.__port)))
        while not self.__stop:
            resp_q=None
            try:
                req_json, resp_q = self.__q.get(timeout=1)
            except Queue.Empty: req_json = None
            if req_json is not None:
                self.__controller.log().LogString("REQ: %s" % json.dumps(req_json))
                try :
                    self.__s.sendto(json.dumps(req_json), (self.__addr, self.__port))
                    retr=self.NRET
                    while retr>0 :
                        d = self.__s.recvfrom(1024)
                        resp_json=json.loads(d[0])
                        try:
                            if int(req_json["I"]) != int(resp_json["I"]) :
                                self.__controller.log().LogString("From %s rsp %s on %s" % (d[1], d[0], str(self.__s.getsockname())), 'GREY')                        
                                self.__controller.log().LogErrorString("UNMATCHED: "+d[0])
                            else:
                                if resp_q is not None:
                                    resp_q.put_nowait(d[0])
                                else:
                                    self.__controller.resp(d[0], req_json)
                                break
                        except KeyError : 
                            self.__controller.log().LogErrorString("Bad resp : %s" % d[0])
                        retr-=1
                        
                    if retr==0 :
                        self.__controller.log().LogErrorString("Retries exceeded")
                        if resp_q is not None: resp_q.put_nowait(None)
                    
                except socket.timeout as msg:
                    self.__controller.log().LogErrorString("Timeout on %s" % str(self.__s.getsockname()))
                    if resp_q is not None: resp_q.put_nowait(None)
                except socket.error as msg:
                    self.__controller.log().LogErrorString("Sock error : %s" % msg)
                
        self.__s.close()
        self.__controller.log().LogString("Cmd thread stopped")

class ListenerThread(threading.Thread):
    # device command-resp communication
    def __init__(self, controller, bindport):
        threading.Thread.__init__(self)
        self.__controller = controller
        self.__bindport=bindport
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
    def __init__(self, controller):
        threading.Thread.__init__(self)
        self.__controller = controller
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

        self.__controller.log().LogString("Start tracking")

        #self.__controller.reqResetMPU()

        while not self.__stop :
            time.sleep(1)
            self.__controller.reqPosition()

        self.__controller.log().LogString("Stop tracking")


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
        #gain_p, gain_d, gain_i, gain_f = 0.5, 5.0, 0.0, 0.02
        gain_p, gain_d, gain_i, gain_f = 2.0, 8.0, 0.05, 5.0
        degain_i=0.5
        err_p_0=0
        err_i=0
        self.complete=False
        s0=0

        self.__controller.reqMoveSync(base_move,base_move)

        while not self.__stop:  #and not within target ....
            resp_json = self.__controller.reqPositionSync()
            try:
                if resp_json is not None and resp_json["C"]=="POS":
                    #time.sleep(0.25) # bad design... wait until map object perform unit location procedure....
                    #x, y, a = self.__unit.x_mean, self.__unit.y_mean, self.__unit.a_mean # by localization
                    x, y, a =self.__unit.GetSim() #by dead reckoning

                    self.__planner.RePlanOnMove((x,y), False)

                    if len(self.__planner.spath)<2 :
                        self.__controller.log().LogString("Got there!")
                        self.complete=True
                        break

                    plan_a=math.atan2( self.__planner.spath[1][0]-self.__planner.spath[0][0],
                                        self.__planner.spath[1][1]-self.__planner.spath[0][1])

                    #self.__controller.log().LogString("After move CRD=(%s %s), A=%s, AP=%s"
                    #                                  % (round(x,2), round(y, 2),
                    #                                     round(a*180/math.pi,2), round(plan_a*180/math.pi,2)))

                    err_p=(a-plan_a)
                    if err_p < -math.pi :
                        err_p += 2*math.pi
                    if err_p > math.pi :
                        err_p -= 2*math.pi


                    err_i=err_p+degain_i*err_i
                    err_d=err_p-err_p_0
                    err_p_0=err_p
                    feedback=-(gain_p*err_p+gain_d*err_d+gain_i*err_i)*gain_f
                    feedback=round(feedback,2)


                    """
                    dmove=[feedback, -feedback]
                    for im in range(2):
                        move_var[im]+=dmove[im]
                        if move_var[im]>move_var_lim: move_var[im]=move_var_lim
                        if move_var[im]<-move_var_lim: move_var[im]=-move_var_lim

                    self.__controller.log().LogString("PID: %s %s %s => L %s => (%s, %s)" %
                                                      (round(err_p,2), round(err_d,2), round(err_i,2), round(feedback,2),
                                                       base_move+move_var[0], base_move+move_var[1]))


                    self.__controller.reqMoveSync(base_move+move_var[0], base_move+move_var[1])
                    """
                    s=-err_p*180/math.pi
                    #if(s<-180) : s=-180
                    #if(s>180) : s=180

                    if(s<-5) : s=-5
                    if(s>5) : s=5

                    s=s0-(s0-s)*0.5 #LPM
                    s0=s


                    self.__controller.log().LogString("PID:  %s" % s)

                    """
                    s=feedback
                    if(s<-100) : s=-100
                    if(s>100) : s=100

                    self.__controller.log().LogString("PID: %s %s %s => %s  => %s" %
                                                      (round(err_p,2), round(err_d,2), round(err_i,2),
                                                       round(feedback,2), round(s,2)))
                    """

                    self.__controller.reqSteerSync(s)

            except KeyError: pass

            #time.sleep(2)
            #time.sleep(0.25) # let it move a bit

        self.__controller.reqMoveSync(0,0)

        self.__controller.log().LogString("Path running thread stopped")