import json
import ConfigParser

class Config(ConfigParser.RawConfigParser) :
    def __init__(self,form,model) :
        ConfigParser.RawConfigParser.__init__(self)
        self.__form = form
        self.__model = model
        self.read('console.cfg')
        try:
            self.__model["DEVADDR"]=str(self.get('DEVICE', 'ADDR'))
            self.__model["DEVPORT"]=int(self.get('DEVICE', 'PORT'))
            self.__model["LISTENPORT"]=int(self.get('DEVICE', 'LISTENPORT'))
            self.__model["SYSLOGENABLE"]=int(self.get('DEVICE', 'SYSLOGENABLE'))
            #self.__model["CMD_HIST"]=self.get('UI', 'CMD_HIST')
            self.__model["CMD_HIST"]=json.loads(self.get('UI', 'CMD_HIST'))
        except ConfigParser.NoSectionError : pass
        except ConfigParser.NoOptionError : pass
        except KeyError : pass


    def update(self) :
        try:
            self.add_section('DEVICE')
        except ConfigParser.DuplicateSectionError : pass
        try:
            self.add_section('UI')
        except ConfigParser.DuplicateSectionError : pass

        self.set('DEVICE', 'ADDR', self.__model["DEVADDR"])
        self.set('DEVICE', 'PORT', self.__model["DEVPORT"])
        self.set('DEVICE', 'LISTENPORT', self.__model["LISTENPORT"])
        self.set('DEVICE', 'SYSLOGENABLE', self.__model["SYSLOGENABLE"])
        #self.set('UI', 'CMD_HIST', self.__model["CMD_HIST"])
        self.set('UI', 'CMD_HIST', json.dumps(self.__model["CMD_HIST"]))
        with open('console.cfg', 'wb') as configfile:
            self.write(configfile)
            configfile.close()
