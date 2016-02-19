'''
    Simple udp socket server
'''
 
import socket
import sys
import random
import json
 
host = ''   # Symbolic name meaning all available interfaces
port = 4444 # Arbitrary non-privileged port

if len(sys.argv) == 2:
    port = int(sys.argv[1])
elif len(sys.argv) > 2:
    host = sys.argv[1]
    port = int(sys.argv[2])
    
#print 'Will listen on'+host+':'+str(port)
 
# Datagram (udp) socket
try :
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(1)
    print 'Socket created'
except socket.error as msg :
    print 'Failed to create socket. Error %s'  % msg
    sys.exit()
 
 
# Bind socket to local host and port
try:
    s.bind((host, port))
except socket.error as msg:
    print 'Bind failed. Error %s' % msg
    sys.exit()
     
print 'Socket bind complete to '+str(s.getsockname())
 
#now keep talking with the client
t=1000
while 1:
    try:
        # receive data from client (data, addr)
        d = s.recvfrom(1024)
        data = d[0]
        addr = d[1]    
        #if not data: 
        #    break   
        #reply = 'OK...' + data   
        #s.sendto(reply , addr)
        print 'Message[' + addr[0] + ':' + str(addr[1]) + '] - ' + data.strip()
        js=json.loads(data.strip())
        # pos: {"C": "I", "T":12345, "R":0, "C": "L", "YPR": [59, 12, 13], "A": [0.01, 0.02, -0.03], "V": [0.1, 0.2, -0.3]}
        if js["C"]=="INFO" : js["FHS"]=99999
        elif js["C"]=="POS" : js.update( {
                "YPR":[int((random.random()-0.5)*360),12,13], 
                "A":[0.01, 0.02, -0.03], 
                "V":[random.random()-0.5, random.random()-0.5, -0.3]})
        js["R"]=0
        js["T"]=t
        #js["I"]=99
        t=t+1000
        s.sendto(json.dumps(js) , addr)
    except KeyError: continue   
    except socket.timeout:
#        print 'Timeout'
        continue    
    except socket.error as msg:
        print 'Rcv/Snd failed. Error %s ' % msg
        sys.exit()
     
s.close()