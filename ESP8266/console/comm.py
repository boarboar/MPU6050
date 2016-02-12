import threading
import Queue
import random
import json
import socket

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
            self.__s.settimeout(1)
        except socket.error:
            self.__controller.log().LogErrorString("Failed to create socket!")
            return

        self.__controller.log().LogString("Cmd thread starting: dev=%s:%s" % (self.__addr, str(self.__port)))
        while not self.__stop:
            while not self.__q.empty():
                req_json = self.__q.get()
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
                            self.__controller.log().LogString("From %s rsp %s" % (d[1], d[0]), 'GREY')
                            resp_json=json.loads(d[0])
                            if self.__controller.resp(d[0], req_json) : break
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
                self.__controller.log().LogString("From %s log %s" % (d[1], d[0]), 'GREY')
                self.__controller.resp(d[0])
            except socket.timeout as msg:
                pass
            except socket.error as msg:
                self.__controller.log().LogErrorString("Sock error : %s" % msg)

        self.__controller.log().LogString("Syslog thread stopped")