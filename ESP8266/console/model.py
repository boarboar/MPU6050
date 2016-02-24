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
        self["YPR"]=[0,0,0]
        self["V"]=[0,0,0]
        self["T"]=0
        self["T_ATT"]=0
        self["MHIST"]=(None,[
                        #{"T":1000, "A":[1, 0, 0], "YPR":[90.0, 0.0, 0.0]},
                        #{"T":2000, "A":[9, 1000, 1], "YPR":[92.0, -10.0, -5.0]}
                        ]
                       )
        self.__lock=threading.Lock()
    def __getitem__(self, key):
        self.__lock.acquire()
        try:
            #value = copy.deepcopy(dict.__getitem__(self, key))
            #value = copy.copy(dict.__getitem__(self, key))
            value = dict.__getitem__(self, key)
        except KeyError: value=None
        finally: self.__lock.release()
        return value

    def update(self, resp_json, reset=False):
        "from sresp"
        update_pos=False
        self.__lock.acquire()
        try:
            if reset==True:
                data=dict.__getitem__(self, "MHIST")
                data[1][:] = [] # clear history
                self["YPR"]=[0,0,0]
                self["V"]=[0,0,0]
                update_pos=True
            self["T_ATT"]=0
            self["T"]=resp_json["T"]
            if resp_json["C"]=="INFO" :
                self["FHS"]=resp_json["FHS"]
                self["FSS"]=resp_json["FSS"]
            elif resp_json["C"]=="POS" or resp_json["C"]=="L":
                #self["X"]=resp_json["X"]
                #self["Y"]=resp_json["Y"]
                data=dict.__getitem__(self, "MHIST")
                if len(data[1])==0  or (int(resp_json["T"]) > int(data[1][-1]["T"])) :
                    item={"T":resp_json["T"], "YPR":resp_json["YPR"], "A":resp_json["A"], "V":resp_json["V"] }
                    data[1].append(item)
                    self["YPR"]=resp_json["YPR"]
                    self["V"]=resp_json["V"]
                    self["T_ATT"]=resp_json["T"]
                    update_pos=True
        except KeyError: pass
        finally: self.__lock.release()
        return update_pos

    def dump(self):
        s=""
        self.__lock.acquire()
        s=pprint.pformat(self, depth=2)
        self.__lock.release()
        return s


