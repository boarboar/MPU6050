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
        #self["PREV_YPR"]=[0,0,0]
        self["V"]=[0,0,0]
        self["CRD"]=[0,0,0] #coord X, Y, Z
        #self["PREV_CRD"]=[0,0,0] #prev coord X, Y, Z
        self["S"]=[0,0,0] #sens  Sl, Sc, Sr
        self["T"]=0
        self["T_ATT"]=0
        self["TURN"]=0 #YAW change
        self["MHIST"]=(None,[
                        #{"T":1000, "V":[0.1, 0, 0], "YPR":[90.0, 0.0, 0.0]},
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
                self["CRD"]=[0,0,0]
                self["S"]=[0,0,0]
                update_pos=True
            self["T_ATT"]=0
            self["T"]=resp_json["T"]
            if resp_json["C"]=="INFO" :
                self["FHS"]=resp_json["FHS"]
                self["FSS"]=resp_json["FSS"]
            elif resp_json["C"]=="POS" or resp_json["C"]=="L":
                data=dict.__getitem__(self, "MHIST")
                if len(data[1])==0  or (int(resp_json["T"]) > int(data[1][-1]["T"])) :
                    item={"T":resp_json["T"], "YPR":resp_json["YPR"],
                          "V":resp_json["V"] }
                    data[1].append(item)
                    #self["PREV_YPR"]=self["YPR"]
                    #self["PREV_CRD"]=self["CRD"]
                    self["YPR"]=resp_json["YPR"]
                    self["V"]=resp_json["V"]
                    self["CRD"]=resp_json["CRD"]
                    self["S"]=resp_json["S"]
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


