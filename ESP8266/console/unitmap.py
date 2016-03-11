import json
import sys
import math
import random

from pprint import pprint

class Particle:
    def __init__(self, x=150, y=200, a=0.0, w=1.0, id=0):
        self.x, self.y = (x, y)
        self.a=a
        self.w=w
        self.id=id

    def move(self, dx, dy, da):
        self.a=self.a+da
        self.x=self.x+dx
        self.y=self.y+dy

    def move_d(self, dist, da):
        self.a=self.a+da
        self.x=self.x+dist*math.sin(self.a)
        self.y=self.y+dist*math.cos(self.a)

    def __repr__(self):
        return 'P_%s <(%s,%s) A %s W %s >' % (str(self.id), str(self.x), str(self.y), str(self.a*180.0/math.pi), str(self.w))

class UnitMap:
    def __init__(self, mapfile):
        self.xu0, self.yu0 = (0, 0)    # unit base (start point)
        self.InitPos()
        self.particles=[]
        self.boundRect=[sys.maxint, sys.maxint, -sys.maxint, -sys.maxint] #bounding rect
        c45=math.cos(math.pi/4)
        self.scan_angles=[-45*math.pi/180.0, 0, 45*math.pi/180.0]
        self.scan_rays=[(-c45, c45), (0, 1.0), (c45, c45)]
        self.scan_max_dist=400
        self.sense_noise=10
        try:
            with open(mapfile) as data_file:
                self.map = json.load(data_file)
            #pprint(__map)
            for area in self.map["AREAS"] :
                for wall in area["WALLS"] :
                    self.adjustBound(wall["C"][0], wall["C"][1])
                    self.adjustBound(wall["C"][2], wall["C"][3])
            self.xu0, self.yu0=self.map["START"]
            print("Map loaded")
            print(self.boundRect)
        #except IOError: pass
        except : pass

    def InitPos(self):
        self.isInside=False
        self.scans=[-1,-1,-1]
        self.__r_cos, self.__r_sin= (1.0, 0.0)    # unit cosine matrix
        self.__r_x, self.__r_y = (0.0, 0.0)    # unit position
        self.__angle=0 #yaw
        self.__r_move=(0.0, 0.0)
        self.__a_rot=0.0
        self.__move=0

    def Reset(self):
        self.InitPos()
        self.InitParticles()

    def InitParticles(self):
        self.particles = [] #clean
        for i in range(4) :
            for j in range(4) :
                a=(random.random()-0.5)*60
                x=(random.random()-0.5)*100+self.xu0
                y=(random.random()-0.5)*100+self.yu0
                self.particles.append(Particle(a=a, x=x, y=y, id=1))

        #self.particles.append(Particle(a=-30.0*math.pi/180.0, id=1))
        #self.particles.append(Particle(a=0, id=2))
        #self.particles.append(Particle(a=30.0*math.pi/180.0, id=3))

    def MoveUnit(self, x, y, angle, scans):
        if self.__move == 0 : # first step
            self.__r_move=(0.0, 0.0)
            self.__a_rot=0.0
            move_dist=0
            # TODO - distribute
            # particles here
        else :
            self.__r_move=(x-self.__r_x, y-self.__r_y)
            self.__a_rot=angle-self.__angle
            if self.__a_rot>math.pi : self.__a_rot=self.__a_rot-math.pi*2
            elif self.__a_rot<-math.pi : self.__a_rot=math.pi*2+self.__a_rot
            move_dist=self.__r_move[0]*self.__r_sin+self.__r_move[1]*self.__r_cos

        print("Unit Mov: %s Rot %s Dist %s" % (str(self.__r_move), str(self.__a_rot*180.0/math.pi), move_dist) )

        self.__r_cos=math.cos(angle)
        self.__r_sin=math.sin(angle)
        self.__r_x=x
        self.__r_y=y
        self.__angle=angle
        self.scans=scans
        self.isInside=self.isInsideTest(x, y)
        self.__move = self.__move+1
        self.updateParticles(move_dist, self.__a_rot, scans)


    def UnitToMap(self, x, y):
        x1=x*self.__r_cos+y*self.__r_sin+self.__r_x+self.xu0
        y1=-x*self.__r_sin+y*self.__r_cos+self.__r_y+self.yu0
        return (x1,y1)

    def adjustBound(self, x, y):
        if x<self.boundRect[0] : self.boundRect[0]=x
        if y<self.boundRect[1] : self.boundRect[1]=y
        if x>self.boundRect[2] : self.boundRect[2]=x
        if y>self.boundRect[3] : self.boundRect[3]=y

    def isInsideTest(self, x, y):
        x, y=(x+self.xu0, y+self.yu0)
        for area in self.map["AREAS"] :
            left, right = (0, 0)
            for wall in area["WALLS"] :
                isect=self.intersectHor(y, wall["C"])
                if isect!=None :
                    if isect<x : left=left+1
                    else : right=right+1
            #print (area["NAME"], left, right)
            if left%2==1 and right%2==1 : return True
        #print ("OUT")
        return False

    def updateParticles(self, mov, rot, scans):
        wsum=0
        for p in self.particles :
            p.move_d(mov, self.__a_rot)
            self.updateParticleProbabilities(p, scans)
            wsum=wsum+p.w
        if wsum>0 :
            for p in self.particles :
                p.w = p.w/wsum
                print(p)

    def getParticleRays(self, p): #just for visual debugging
        rays=[]
        p0=(p.x, p.y)
        for a in self.scan_angles :
            p1=(p.x+math.sin(p.a+a)*self.scan_max_dist, p.y+math.cos(p.a+a)*self.scan_max_dist)
            rays.append((p0, p1))
        return rays

    def getParticleIntersects(self, p): #just for visual debugging
        intersects=[]
        p0=(p.x, p.y)
        for a in self.scan_angles :
            p1=(p.x+math.sin(p.a+a)*self.scan_max_dist, p.y+math.cos(p.a+a)*self.scan_max_dist)
            dist2=0
            intrs=None
            for area in self.map["AREAS"] :
                for wall in area["WALLS"] :
                    p2=([wall["C"][0], wall["C"][1]])
                    p3=([wall["C"][2], wall["C"][3]])
                    isect=self.find_intersection(p0, p1, p2, p3)
                    if isect!=None :
                        d2 = (isect[0]-p0[0])*(isect[0]-p0[0])+(isect[1]-p0[1])*(isect[1]-p0[1])
                        if intrs==None or d2<dist2 :
                            intrs=isect
                            dist2=d2
            intersects.append(intrs)
        return intersects

    def updateParticleProbabilities(self, p, meas):
        scan_dist=[]
        prob = 1.0;
        p0=(p.x, p.y)
        #for a in self.scan_angles :
        for i in range(len(self.scan_angles)) :
            a=self.scan_angles[i]
            p1=(p.x+math.sin(p.a+a)*self.scan_max_dist, p.y+math.cos(p.a+a)*self.scan_max_dist)
            dist2=0
            intrs=None
            for area in self.map["AREAS"] :
                for wall in area["WALLS"] :
                    p2=([wall["C"][0], wall["C"][1]])
                    p3=([wall["C"][2], wall["C"][3]])
                    isect=self.find_intersection(p0, p1, p2, p3)
                    if isect!=None :
                        d2 = (isect[0]-p0[0])*(isect[0]-p0[0])+(isect[1]-p0[1])*(isect[1]-p0[1])
                        if intrs==None or d2<dist2 :
                            intrs=isect
                            dist2=d2
            if intrs==None : dist2 = -1
            else : dist2=math.sqrt(dist2)
            scan_dist.append(dist2)
            prob*=self.Gaussian(dist2, self.sense_noise, meas[i])
        p.w=prob
        #print(scan_dist)

    def getIntersection(self, x0, y0, x1, y1):
        # ray is (0.0)->(x,y) line
        # assume that we are inside
        # get a closest one to starting pt
        #intrs = []
        intrs = None
        dist2=0
        p0=self.UnitToMap(x0, y0)
        p1=self.UnitToMap(x1, y1)

        for area in self.map["AREAS"] :
            for wall in area["WALLS"] :
                p2=([wall["C"][0], wall["C"][1]])
                p3=([wall["C"][2], wall["C"][3]])
                isect=self.find_intersection(p0, p1, p2, p3)
                if isect!=None :
                    d2 = (isect[0]-p0[0])*(isect[0]-p0[0])+(isect[1]-p0[1])*(isect[1]-p0[1])
                    if intrs==None or d2<dist2 :
                        intrs=isect
                        dist2=d2
        #print (intrs)
        return intrs

    def intersectHor(self, y, line):
        x0, y0, x1, y1 = line
        if y0==y1 : #with hor line
            if y!=y0 : return None # no intersect
            else : return None # SPECIAL CASE :: OM LINE (TODO)
        if (y<y1 and y<=y0) or (y>y1 and y>=y0) : return None  # no intersect, note - left point not included (?)

        if x0>x1 : x0, y0, x1, y1 = x1, y1, x0, y0 #reorder

        x = x0+(y-y0)/(y1-y0)*(x1-x0)
        return x


    def find_intersection(self,  p0, p1, p2, p3 ) :
        s10_x = p1[0] - p0[0]
        s10_y = p1[1] - p0[1]
        s32_x = p3[0] - p2[0]
        s32_y = p3[1] - p2[1]
        denom = s10_x * s32_y - s32_x * s10_y
        if denom == 0 : return None # collinear
        denom_is_positive = denom > 0
        s02_x = p0[0] - p2[0]
        s02_y = p0[1] - p2[1]
        s_numer = s10_x * s02_y - s10_y * s02_x
        if (s_numer < 0) == denom_is_positive : return None # no collision
        t_numer = s32_x * s02_y - s32_y * s02_x
        if (t_numer < 0) == denom_is_positive : return None # no collision
        if (s_numer > denom) == denom_is_positive or (t_numer > denom) == denom_is_positive : return None # no collision
        # collision detected
        t = t_numer / denom
        intersection_point = ( int(p0[0] + (t * s10_x)), int(p0[1] + (t * s10_y)) )
        return intersection_point

    def Gaussian(self, mu, sigma, x):
        # calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
        if mu < 0 : mu=self.scan_max_dist
        if x < 0 : x=self.scan_max_dist
        return math.exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / math.sqrt(2.0 * math.pi * (sigma ** 2))