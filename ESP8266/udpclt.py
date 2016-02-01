'''
    udp socket client
'''
 
import socket   #for sockets
import sys  #for exit
 
# create dgram udp socket
try:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(1)
except socket.error:
    print 'Failed to create socket'
    sys.exit()
 
host = 'localhost';
port = 4444;

if len(sys.argv) > 1:
    host = sys.argv[1]
if len(sys.argv) > 2:
    port = int(sys.argv[2])
    
print 'Will connect to'+host+':'+str(port)
 
while(1) :
    msg = raw_input('Enter message to send : ')
    
    if msg=='bye':
        sys.exit(0)
     
    try :
        #Set the whole string
        s.sendto(msg, (host, port))         
        # receive data from client (data, addr)
        d = s.recvfrom(1024)
        reply = d[0]
        addr = d[1]
         
        print 'Server reply : ' + reply
     
    except socket.error, msg:
        print 'Error Code : ' + str(msg[0]) + ' Message ' + msg[1]
        sys.exit()