'''
    Simple udp socket server
'''
 
import socket
import sys
import random
import math
import json

sys.path.insert(0, './console')
import unitmap
import unit

__r_cos=1
__r_sin=0
__r_x=0
__r_y = 0 # simulated crd
isInside=False
def gauss_lim(m, d, l) :

    v=random.gauss(m, d)
    if v<-l: v=-l
    if v>l: v=l
    return v

def MoveUnit(angle, dist, scans, x, y):
    global __r_cos
    global __r_sin
    global __r_x
    global __r_y
    global isInside
    __r_cos=math.cos(angle)
    __r_sin=math.sin(angle)
    __r_x, __r_y = x, y # simulated crd
    isInside=map.isInsideTest(x+map.init_start[0], y+map.init_start[1])

def UnitToMapSim(x, y):
    global __r_cos
    global __r_sin
    global __r_x
    global __r_y
    global isInside
    x1=x*__r_cos+y*__r_sin+__r_x+map.init_start[0]
    y1=-x*__r_sin+y*__r_cos+__r_y+map.init_start[1]
    return (x1,y1)

    
#json.encoder.FLOAT_REPR = lambda o: format(o, '.2f')
   
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
 
map =  unitmap.UnitMap('./console/map.json')
unit =  unit.Unit(map, None, 400)
#unit.InitUnitPos(map.start)

#now keep talking with the client
t=1000
yaw=0
rx, ry = (0, 0)
drive=[0, 0]
corr=0
targ_yaw=0
init =True
WHEEL_RAD=3.5
WHEEL_BASE=13

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
        elif js["C"]=="D" : #drive
            if "RPS" in js :
                drive=js["RPS"]                                            
                print ("Driving %s" % str(drive))
                if drive[0]==0 and drive[1]==0 :
                    targ_yaw=0
                    vx=0
                    vy=0
                    rvx=0
                    rvy=0
                    vv=0
            #js["ARPS"]=[round(drive[0]/2,2), round(drive[1]/2,2)]
            js["W"]=[round(drive[0]*100/2,2), round(drive[1]*100/2,2)]
            js["TW"]=[round(drive[0]*100/2,2), round(drive[1]*100/2,2)]
        elif js["C"]=="M" : #move
            if "V" in js : 
                drive=[js["V"]/10, js["V"]/10]                
                print ("Move/Driving %s" % str(drive))
        elif js["C"]=="S" : #steer
            if "S" in js : 
                targ_yaw=yaw+js["S"]           
                print ("Steering %s" % str(js["S"]))
        elif js["C"]=="RSTMPU" : 
            start=js["P"]
            print start
            #map.SetStartPoint(start)
            map.init_start=start
            init=True
        elif js["C"]=="POS" : 
            if init : 
                yaw=0
                rx=0
                ry=0
                vx=0
                vy=0
                rvx=0
                rvy=0
                vv=0
                dmov=0
                targ_yaw=0
                init=False
            else :   
            
                lrmov=[d*2.0*math.pi*WHEEL_RAD*(1.0+gauss_lim(0, 0.1, 0.5)) for d in drive]
                print ("MovedLR %s" % str(lrmov))
                
                cmov=(lrmov[0]+lrmov[1])*0.5;                
                crot=(lrmov[0]-lrmov[1])/WHEEL_BASE;                
  
                yaw += crot*180.0/math.pi
  
                #yaw = yaw + gauss_lim(0, 20, 20) #tends to turn right
                #yaw = yaw + gauss_lim(-10, 10, 20) #tends to turn left
                
                #yaw = yaw + corr
                
                yaw = yaw + (targ_yaw-yaw)/2
                
                yaw=int(yaw)%360
                if yaw>180 : yaw = yaw-360
                
                #dvy= yaw+gauss_lim(0, 2, 4)
                #vx = math.cos(dvy*math.pi/180.0)*cmov #in X-UP/Y-LEFT SYSTEM
                #vy =-math.sin(dvy*math.pi/180.0)*cmov       

                #vv=math.hypot(vx, vy)
                
                #in X-RIGHT/Y-UP SYSTEM
                
                rvx=cmov*math.sin(yaw*math.pi/180.0)
                rvy=cmov*math.cos(yaw*math.pi/180.0)
                
                rx=rx+rvx #CM
                ry=ry+rvy #CM  
                
                dmov+=cmov
                
            MoveUnit(yaw*math.pi/180.0, dmov, [-1,-1,-1], rx, ry)
            mapx, mapy = UnitToMapSim(0, 0)
            print mapx, mapy
            
            intrsects = []
            intrsects0 = []
            #sorted_walls=map.getSortedWalls((mapx, mapy), unit.scan_max_dist)
            sorted_walls=map.getSortedWalls((mapx, mapy))
            
            for a in unit.scan_angles :
                #intrs0, pr, intrs1, refstate, intrs=map.getIntersectionUnit(0, 0, math.sin(a)*map.scan_max_dist, rvy+math.cos(a)*map.scan_max_dist)  
                """
                intrs0, pr, intrs1, refstate, intrs, cosa2, dist =map.getIntersectionMapRefl(
                    (mapx, mapy), 
                    UnitToMapSim(math.sin(a)*unit.scan_max_dist, rvy+math.cos(a)*unit.scan_max_dist),
                    unit.scan_max_dist, sorted_walls)  
                
                if intrs!=None :
                    intrs=(intrs[0]-mapx, intrs[1]-mapy)                                        
                    dist=math.sqrt(intrs[0]*intrs[0]+intrs[1]*intrs[1])
                    err=gauss_lim(0, dist/10, 150) 
                    print ("DIST %s ERR %s" % (str(dist), str(err)))
                    dist=dist+err
                    if dist<0 : dist=0
                    dist=round(dist, 2)
                else :
                    dist=-1
                intrsects.append(dist)                

                if intrs0!=None :
                    intrs0=(intrs0[0]-mapx, intrs0[1]-mapy)                                        
                    dist0=math.sqrt(intrs0[0]*intrs0[0]+intrs0[1]*intrs0[1])
                else :
                    dist0=-1
                intrsects0.append(dist0)   
                """
                dist = map.getIntersectionMapClean((mapx, mapy), 
                    UnitToMapSim(math.sin(a)*unit.scan_max_dist, rvy+math.cos(a)*unit.scan_max_dist), sorted_walls)   
                if dist is not None:
                    err=gauss_lim(0, dist/10, 150) 
                    dist=dist+err
                    if dist<0 : dist=0
                    dist=round(dist, 2)
                else:     
                    dist = -1
                    
                intrsects.append(dist)                
            print(intrsects)
            sr=len(intrsects)-1
            sm=(len(intrsects)-1)/2
            
            """
            if not isInside : corr=180
            elif intrsects0[sm]>=0 and intrsects0[sm]<90:
                #value=10
                #if intrsects[sm]<20 : value=90
                #elif intrsects[sm]<40 : value=45    
                #elif intrsects[sm]<60 : value=30  
                value=(100.0-intrsects0[sm])
                if intrsects0[0]<intrsects0[sr] : corr=value
                else : corr=-value
                print ("Correct with %s" % corr)
            else : corr=0
            """
            
            js.update( {
                "YPR":[round(yaw, 2),int((random.random()-0.5)*360),int((random.random()-0.5)*360)], 
                #"V":[round(rvx,2), round(rvy,2), 0.0],
                #"V":[0, 0, 0],
                "V":round((rvx+rvy)/2,2),
                "CRD":[round(rx,2), round(ry,2), 0],
                "D":round(dmov,2),
                "W":[round(drive[0]*100/2,2), round(drive[1]*100/2,2)],
                "S":intrsects
                })
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


    