import threading
import Queue
import random
import json
import socket
import time
import math
import timeit


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
            start_time = timeit.default_timer()
            self.__controller.reqPositionSync()
            t=timeit.default_timer() - start_time
            if t>0.8 : time.sleep(0.2)
            else : time.sleep(1-t)
            self.__controller.log().InfoString(str(round(t, 2)))
            #time.sleep(1.0)

        self.__controller.log().LogString("Stop tracking")


class PathThread(threading.Thread):
    # device command-resp communication
    def __init__(self, controller, planner, unit, speed, resume_track):
        threading.Thread.__init__(self)
        self.__controller = controller
        self.__planner = planner
        self.__unit = unit
        self.__stop = False
        self.__speed = speed
        self.__resume_track = resume_track
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
        closing=False

        self.__controller.reqMoveSpeedSync(self.__speed) #cm/s

        while not self.__stop :
            resp_json = self.__controller.reqPositionSync()
            try:
                if resp_json is not None and resp_json["C"]=="POS":
                    x, y, a = self.__unit.x_mean, self.__unit.y_mean, self.__unit.a_mean # by localization
                    #x, y, a =self.__unit.GetSim() #by dead reckoning

                    self.__planner.RePlanOnMove((x,y), False)

                    if len(self.__planner.spath) < 4:
                        self.__controller.log().LogString("Got there!")
                        self.complete=True
                        break

                    if len(self.__planner.spath) < 8 and not closing:
                        self.__controller.log().LogString("Closing++++++++++++++++++++++")
                        self.__controller.reqMoveSpeedSync(self.__speed/2)  #cm/s
                        closing = True

                    plan_a=math.atan2( self.__planner.spath[1][0]-self.__planner.spath[0][0],
                                        self.__planner.spath[1][1]-self.__planner.spath[0][1])

                    #self.__controller.log().LogString("After move CRD=(%s %s), A=%s, AP=%s"
                    #                                  % (round(x,2), round(y, 2),
                    #                                     round(a*180/math.pi,2), round(plan_a*180/math.pi,2)))

                    ## apply LPM
                    x_a = math.sin(plan_a) + math.sin(a)
                    y_a = math.cos(plan_a) + math.cos(a)
                    a_a = math.atan2(y_a, x_a)
                    a_a = math.pi * 0.5 - a_a
                    if a_a < -math.pi:
                        a_a += 2 * math.pi
                    if a_a > math.pi:
                        a_a -= 2 * math.pi

                    print("Plan run: A_LOC=%s, A_REQ=%s, A_LPM=%s" %
                          (round(a*180/math.pi, 0), round(plan_a*180/math.pi), round(a_a*180/math.pi, 0)))

                    #self.__controller.reqBearingSync(plan_a*180/math.pi)
                    self.__controller.reqBearingSync(a_a * 180 / math.pi)

            except KeyError: pass

            #time.sleep(0.25) # let it move a bit

        self.__controller.reqMoveSpeedSync(0)
        self.__controller.log().LogString("Path running thread stopped")
        if self.__resume_track :
            self.__controller.startSimulation()
