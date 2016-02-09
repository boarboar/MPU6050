import threading
import Queue
import random
import json

class CommandThread(threading.Thread):
    # device command-resp communication
    def __init__(self,controller):
        threading.Thread.__init__(self)
        self.__controller = controller
        self.__q = Queue.Queue()
        self.setDaemon(1)
    def put(self, msg):
        self.__q.put_nowait(msg)
    def run (self):
      self.__controller.log().LogString("Thread started")
      while 1:
          while not self.__q.empty():
                message = self.__q.get()
                self.__controller.log().LogString("REQ:"+message)
                #
                #
                try:
                    req_json = json.loads(message)
                    if req_json["C"]=="INFO" : rsp_js = json.dumps({"C": "INFO", "FHS": (int)(random.random()*40000), "FSS": (int)(random.random()*200000)})
                    elif req_json["C"]=="POS" : rsp_js = json.dumps({"C": "POS", "X": random.random(), "Y": random.random()})
                    else : rsp_js = ""
                    self.__controller.resp(rsp_js)
                except KeyError: continue
