import json
import threading
import copy

class Model(dict):
    def __init__(self, name=None):
        self["NAME"] = name
        self.__lock=threading.Lock()
    def __getitem__(self, key):
        self.__lock.acquire()
        try:
            value = copy.deepcopy(dict.__getitem__(self, key))
        except KeyError: value=None
        finally: self.__lock.release()
        return value

    def update(self, js):
        resp_json = json.loads(js)
        self.__lock.acquire()
        try:
            if resp_json["C"]=="INFO" :
                self["FHS"]=resp_json["FHS"]
                self["FSS"]=resp_json["FSS"]
            elif resp_json["C"]=="POS" :
                self["X"]=resp_json["X"]
                self["Y"]=resp_json["Y"]
        except KeyError: pass
        finally: self.__lock.release()



