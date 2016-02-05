import threading
import Queue
import random
import json

class CommandThread(threading.Thread):
    # device command-resp communication
    def __init__(self,form):
        threading.Thread.__init__(self)
        self.__form = form
        self.__q = Queue.Queue()
        self.setDaemon(1)
    def put(self, msg):
        self.__q.put_nowait(msg)
    def run (self):
      self.__form.LogString("Thread started")
      while 1:
          while not self.__q.empty():
                message = self.__q.get()
                self.__form.LogString(message)
                js = json.dumps({"x": random.random(), "y": random.random()})
                self.__form.Update("resp", result='succ', data=js)
