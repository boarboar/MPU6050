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
        self.__port=port
        self.__mockup=mockup
        self.__stop = False
        self.setDaemon(1)
    def put(self, msg):
        self.__q.put_nowait(msg)
    def stop(self) : self.__stop=True
    def run (self):
        self.__controller.log().LogString("Cmd thread starting: dev=%s:%s" % (self.__addr, str(self.__port)))
        while not self.__stop:
            while not self.__q.empty():
                message = self.__q.get()
                self.__controller.log().LogString("REQ:"+message)
                    #
                    #
                if self.__mockup :
                    try:
                        req_json = json.loads(message)
                        if req_json["C"]=="INFO" : rsp_js = json.dumps({"C": "INFO", "FHS": int(random.random()*40000), "FSS": (int)(random.random()*200000)})
                        elif req_json["C"]=="POS" : rsp_js = json.dumps({"C": "POS", "X": int(random.random()*100), "Y": int(random.random()*100)})
                        else : rsp_js = ""
                        self.__controller.resp(rsp_js)
                    except KeyError: continue

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
        self.__controller.log().LogString("Listener thread starting on %s" % (str(self.__bindport)))
        while not self.__stop:
            #
            #
            #
            continue

        self.__controller.log().LogString("Listener thread stopped")