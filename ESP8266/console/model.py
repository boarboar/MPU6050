import json
import threading
import copy
import socket
import pprint

class Model(dict):
    def __init__(self, name=None, mock=False):
        self["NAME"] = name
        self["MOCKUP"] = mock
        #self["DEVADDR"] = "192.168.1.129"
        self["DEVADDR"] = str(socket.gethostbyname(socket.gethostname()))
        self["DEVPORT"] = 4444
        self["LISTENPORT"] = 4444
        self["SYSLOGENABLE"] = 1
        self["FHS"]=0
        self["FSS"]=0
        #self["X"]=0
        #self["Y"]=0
        self["YPR"]=[0,0,0]
        self["MHIST"]=(2,[
                        {"T":1000, "Q":[1.0, 0.0, 0.0, 1.0], "YPR":[90.0, 0.0, 0.0]},
                        {"T":2000, "Q":[0.9, 0.0, 0.1, -0.9], "YPR":[92.0, -10.0, -5.0]}
                        ]
                       )
        self.__lock=threading.Lock()
    def __getitem__(self, key):
        self.__lock.acquire()
        try:
            value = copy.deepcopy(dict.__getitem__(self, key))
        except KeyError: value=None
        finally: self.__lock.release()
        return value
    def update(self, resp_json):
        "from sresp"
        self.__lock.acquire()
        try:
            if resp_json["C"]=="INFO" :
                self["FHS"]=resp_json["FHS"]
                self["FSS"]=resp_json["FSS"]
            elif resp_json["C"]=="POS" :
                #self["X"]=resp_json["X"]
                #self["Y"]=resp_json["Y"]
                self["YPR"]=resp_json["YPR"]
        except KeyError: pass
        finally: self.__lock.release()

    def update_log(self, resp_json):
        "from syslog"
        self.__lock.acquire()
        try:
            item={"T":resp_json["T"], "Q":resp_json["Q"], "YPR":resp_json["YPR"]}
            self["MHIST"][1].append()
            self["MHIST"][0]=len(self["MHIST"][1])
        except KeyError: pass
        finally: self.__lock.release()

    def dump(self):
        s=""
        self.__lock.acquire()
        s=pprint.pformat(self, depth=2)
        self.__lock.release()
        return s


