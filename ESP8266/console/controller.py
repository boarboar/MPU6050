import comm
import json

import model

class Controller():
    # device controller
    def __init__(self,form,model):
        self.__form = form
        self.__model = model
        self.__cmdid=0
        self.__tstart()

    def __tstart(self):
        try:
            self.__comm_thread=comm.CommandThread(self, self.__model["DEVADDR"], self.__model["DEVPORT"], self.__model["MOCKUP"])
            self.__comm_thread.start()
            self.__comm_listener_thread=comm.ListenerThread(self, self.__model["LISTENPORT"], self.__model["MOCKUP"])
            self.__comm_listener_thread.start()
        except KeyError: pass

    def log(self): return self.__form

    def restart(self):
        self.__comm_listener_thread.stop()
        self.__comm_thread.stop()
        self.__comm_thread.join()
        self.__comm_listener_thread.join()
        self.__form.LogString("Restarting...")
        self.__tstart()

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
        #if resp_json["C"]=="POS" :
        self.__form.UpdatePos()

