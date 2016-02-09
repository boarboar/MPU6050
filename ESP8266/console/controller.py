import comm
import json

import model

class Controller():
    # device controller
    def __init__(self,form,model):
        self.__form = form
        self.__model = model
        self.__cmdid=0
        self.__comm_thread=comm.CommandThread(self)
        self.__comm_thread.start()

    def log(self): return self.__form

    def reqStatus(self):
        # {"I":1,"C":"INFO"}
        self.__cmdid=self.__cmdid+1
        js = json.dumps({"I": self.__cmdid, "C": "INFO"})
        self.__comm_thread.put(js)

    def reqPosition(self):
        # {"I":1,"C":"POS"}
        self.__cmdid=self.__cmdid+1
        js = json.dumps({"I": self.__cmdid, "C": "POS"})
        self.__comm_thread.put(js)

    def resp(self, js):
        " resp callback, called in thread context!!! "
        self.__form.LogString("RSP:"+js)
        self.__model.update(js)
        resp_json = json.loads(js)
        if resp_json["C"]=="POS" : self.__form.UpdatePos()

