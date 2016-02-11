import comm
import json
import socket
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

    def reqCmdRaw(self, cmd):
        try:
            js=json.loads(cmd)
            self.__req(js)
        except ValueError: pass

    def reqStatus(self):
        # {"I":1,"C":"INFO"}
        self.__req({"C": "INFO"})

    def reqPosition(self):
        # {"I":1,"C":"POS"}
        self.__req({"C": "POS"})

    def reqUpload(self):
        # config upload
        # {"I":1,"C":"SYSL", "ON":1, "ADDR":"192.168.1.141", "PORT":4444}
        js={"C": "SYSL"}
        js["ON"]=1
        js["ADDR"]=str(socket.gethostbyname(socket.gethostname()))
        js["PORT"]=int(self.__model["LISTENPORT"])
        self.__req(js)

    def __req(self, js):
        self.__cmdid=self.__cmdid+1
        js["I"]=self.__cmdid
        self.__comm_thread.put(js)

    def resp(self, js, req_json=None):
        " resp callback, called in thread context!!! "
        self.__form.LogString("RSP:"+js)
        try:
            resp_json = json.loads(js)
            if(req_json is not None and int(req_json["I"]) != int(resp_json["I"])) :
                self.__form.LogString("Unmatched resp")
                return False
        except ValueError : return True
        except KeyError : return True
        self.__model.update(resp_json)
        #if resp_json["C"]=="POS" :
        self.__form.UpdatePos()
        return True

