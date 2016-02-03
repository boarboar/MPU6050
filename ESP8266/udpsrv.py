'''
    Simple udp socket server
'''
 
import socket
import sys
 
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
    except socket.timeout:
#        print 'Timeout'
        continue
    except socket.error as msg:
        print 'Rcv/Snd failed. Error %s ' % msg
        sys.exit()
     
s.close()