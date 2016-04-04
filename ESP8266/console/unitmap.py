import json
import sys
import math
import random
import copy

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
        scan_a0=-90
        scan_n=3
        scan_d=(-scan_a0*2)/(scan_n-1)
        self.scan_angles=[]
        self.scan_rays=[]
        for i in range(scan_n) :
            a=(scan_a0+i*scan_d)*math.pi/180.0
            self.scan_angles.append(a)
            self.scan_rays.append((math.sin(a),math.cos(a)))

        self.scan_max_dist=400
        self.sense_noise=20
        self.fwd_noise=5
        self.rot_noise=0.5
        try:
            with open(mapfile) as data_file:
                self.map = json.load(data_file)
            #pprint(__map)
            for area in self.map["AREAS"] :
                parea0=area["AT"]
                for wall in area["WALLS"] :
                    self.adjustBound(parea0[0]+wall["C"][0], parea0[1]+wall["C"][1])
                    self.adjustBound(parea0[0]+wall["C"][2], parea0[1]+wall["C"][3])
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
        self.__dist=0

        self.__move_step=0
        self.__rdist=0
        self.x_mean, self.y_mean, self.p_var, self.a_mean, self.a_var = (0,0,0,0,0)

    def Reset(self):
        self.InitPos()
        self.InitParticles()

    def InitParticles(self):
        N_D=20
        W=1.0/(N_D*N_D)
        LOC_VAR=150
        ANG_VAR=math.pi/2
        self.particles = [] #clean
        for i in range(N_D) :
            for j in range(N_D) :
                a=(random.random()-0.5)*ANG_VAR*2
                x=(random.random()-0.5)*LOC_VAR+self.xu0
                y=(random.random()-0.5)*LOC_VAR+self.yu0
                self.particles.append(Particle(a=a, x=x, y=y, id=i*N_D+j, w=W))

    def MoveUnit(self, x, y, angle, dist, scans):
        if self.__move_step == 0 : # first step
            move_rot=0
            move_dist=0
        else :
            move_rot=angle-self.__angle
            if move_rot>math.pi : move_rot=move_rot-math.pi*2
            elif move_rot<-math.pi : move_rot=math.pi*2+move_rot
            move_dist=dist-self.__dist

        print("Unit Mov: Rot %s Dist %s " % (str(move_rot*180.0/math.pi), move_dist) )

        self.__r_cos=math.cos(angle)
        self.__r_sin=math.sin(angle)
        self.__r_x=x
        self.__r_y=y
        self.__angle=angle
        self.__dist=dist
        self.scans=scans
        self.isInside=self.isInsideTest(x+self.xu0, y+self.yu0)
        self.__move_step = self.__move_step+1
        if len(self.particles)>0 :
            self.updateParticles(move_dist, move_rot, scans)


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
        for area in self.map["AREAS"] :
            left, right = (0, 0)
            for wall in area["WALLS"] :
                parea0=area["AT"]
                wall_crd=wall["C"]
                isect=self.intersectHor(y, parea0[0]+wall_crd[0], parea0[1]+wall_crd[1],
                                        parea0[0]+wall_crd[2], parea0[1]+wall_crd[3])
                if isect!=None :
                    if isect<x : left=left+1
                    else : right=right+1
            #print (area["NAME"], left, right)
            if left%2==1 and right%2==1 : return True
        #print ("OUT")
        return False

    def updateParticles(self, mov, rot, scans):

        for p in self.particles :
            p.move_d(mov+random.gauss(0, self.fwd_noise), rot+random.gauss(0, self.rot_noise))
            if self.isInsideTest(p.x, p.y) :
                self.updateParticleProbabilities(p, scans)
            else : p.w=0.0
        #print self.particles
        mw=max(p.w for p in self.particles)
#        if self.__move < 5 or self.__move%5==0 :
        if True :
            #resmple
            # use algorithm from udacity
            #print("Resample")
            N=len(self.particles)
            p3 = []
            index = int(random.random() * N)
            beta = 0.0
            for i in range(N):
                beta += random.random() * 2.0 * mw
                while beta > self.particles[index].w:
                    beta -= self.particles[index].w
                    index = (index + 1) % N
                p3.append(copy.copy(self.particles[index]))
            self.particles = p3
            #print self.particles

        #normalize
        wsum=sum(p.w for p in self.particles)
        if wsum>0 :
            for p in self.particles :
                p.w = p.w/wsum

        #print self.particles
        self.x_mean, self.y_mean, self.p_var, self.a_mean, self.a_var = self.getMeanDistribution()

    def getMeanDistribution(self):
        x,y, var=0,0,0
        a, vara=0, 0
        for p in self.particles :
            x+=p.x*p.w
            y+=p.y*p.w
            a+=p.a*p.w

        for p in self.particles :
            ex=p.x-x
            ey=p.y-y
            var += (ex*ex+ey*ey)*p.w
            ea=p.a-a
            vara+=ea*ea*p.w

        var= math.sqrt(var)
        vara= math.sqrt(vara)
        #print("Variance %s" % str(round(var,2)))
        return (x, y, var, a, vara)

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
            intrs, ref = self.getIntersectionMap(p0, p1)
            intersects.append(intrs)
        return intersects

    def updateParticleProbabilities(self, p, meas):
        scan_dist=[]
        prob = 1.0;
        p0=(p.x, p.y)
        for i in range(len(self.scan_angles)) :
            a=self.scan_angles[i]
            p1=(p.x+math.sin(p.a+a)*self.scan_max_dist, p.y+math.cos(p.a+a)*self.scan_max_dist)
            intrs0, pr, intrs1, refstate, intrs = self.getIntersectionMapRefl(p0, p1)
            if intrs==None : dist2 = -1
            else :
                dist2=math.sqrt((intrs[0]-p0[0])*(intrs[0]-p0[0])+(intrs[1]-p0[1])*(intrs[1]-p0[1]))
            scan_dist.append(dist2)
            prob*=self.Gaussian(dist2, self.sense_noise, meas[i])
        p.w=prob
        #print(scan_dist)

    def getIntersection(self, x0, y0, x1, y1, findRefl=False):
        # ray is (0.0)->(x,y) line
        # assume that we are inside
        # get a closest one to starting pt
        p0=self.UnitToMap(x0, y0)
        p1=self.UnitToMap(x1, y1)
        return self.getIntersectionMap(p0, p1, findRefl)

    def getIntersectionUnit(self, x0, y0, x1, y1, findRefl=False):
        # helper to input Unit coords
        # ray is (0.0)->(x,y) line
        # assume that we are inside
        # get a closest one to starting pt
        p0=self.UnitToMap(x0, y0)
        p1=self.UnitToMap(x1, y1)
        return self.getIntersectionMapRefl(p0, p1)

    def getIntersectionMapRefl(self, p0, p1):
        intrs0, ref=self.getIntersectionMap(p0, p1, True)
        refState = False
        pr = None
        intrs1 = None
        intrs = None
        if intrs0 != None :
            refState = False
            intrs=intrs0
            if ref != None :
                pr, cosa, refState = ref
            if refState :
                # secondary intersect if any
                intrs1, ref=self.getIntersectionMap(intrs0, pr, False)
                intrs=intrs1
        return (intrs0, pr, intrs1, refState, intrs)

    def getIntersectionMap(self, p0, p1, findRefl=False):
        # line p0->p1 in absolute map coords (world)
        intrs = None
        ref=None
        dist2=0
        reff=0

        for area in self.map["AREAS"] :
            parea0=area["AT"]
            for wall in area["WALLS"] :
                isect=None
                opened=0
                wall_crd=wall["C"]
                p2=(parea0[0]+wall_crd[0], parea0[1]+wall_crd[1])
                p3=(parea0[0]+wall_crd[2], parea0[1]+wall_crd[3])
                if 'S' in wall : opened=wall["S"]
                if opened==0 or (opened==2 and random.random()>0.5):
                    isect=self.find_intersection(p0, p1, p2, p3)
                    #isect=None
                if isect!=None :
                    d2 = (isect[0]-p0[0])*(isect[0]-p0[0])+(isect[1]-p0[1])*(isect[1]-p0[1])
                    if intrs==None or d2<dist2 :
                        intrs=isect
                        dist2=d2
                        wsect=(p2, p3)
                        reff=area["WALLSREFL"]

            #continue

            try:
                for obj in area["OBJECTS"] :
                    if len(obj["CS"])<2 or obj["DENSITY"] < 0.1 : continue
                    pobj0=(parea0[0]+obj["AT"][0], parea0[1]+obj["AT"][1])

                    """
                    #test insideness, and skip object if inside!
                    left, right = (0, 0)
                    op0=obj["CS"][-1]["C"]

                    for c in obj["CS"] :
                        op=c["C"]
                        isect=self.intersectHor(p0[1], parea0[0]+wall["C"][0], parea0[1]+wall["C"][1],
                                        parea0[0]+wall["C"][2], parea0[1]+wall["C"][3])
                        if isect!=None :
                            if isect<p0[0] : left=left+1
                            else : right=right+1
                    if left%2==1 and right%2==1 :
                        #if inside - skip intersecting
                        print ("Hmm...somehow got inside the object...")
                        continue
                    """

                    op0=obj["CS"][-1]["C"]
                    for c in obj["CS"] :
                        op=c["C"]
                        isect=self.find_intersection(p0, p1, (pobj0[0]+op0[0], pobj0[1]+op0[1]), (pobj0[0]+op[0], pobj0[1]+op[1]))
                        #isect=None
                        if isect!=None :
                            d2 = (isect[0]-p0[0])*(isect[0]-p0[0])+(isect[1]-p0[1])*(isect[1]-p0[1])
                            if intrs==None or d2<dist2 :
                                intrs=isect
                                dist2=d2
                                wsect=((pobj0[0]+op0[0], pobj0[1]+op0[1]), (pobj0[0]+op[0], pobj0[1]+op[1]))
                                reff=0
                        op0=op
            except KeyError :
                print('Some obj attr missing')
                pass


        #return intrs, ref

        if intrs!=None and findRefl :
            # find reflection vector
            rl=self.scan_max_dist-math.sqrt(dist2)
            #if rl>0 :
            if rl<=0 : rl=10
            ref=self.getReflection(p0, intrs, wsect[0], wsect[1], rl, reff)

        return intrs, ref

    def intersectHor(self, y, x0, y0, x1, y1):
        if y0==y1 : #with hor line
            if y!=y0 : return None # no intersect
            else : return None # SPECIAL CASE :: OM LINE (TODO)
        if (y<y1 and y<=y0) or (y>y1 and y>=y0) : return None  # no intersect, note - left point not included (?)

        if x0>x1 : x0, y0, x1, y1 = x1, y1, x0, y0 #reorder

        x = x0+(y-y0)/(y1-y0)*(x1-x0)
        return x

    def getReflection(self, p0, p1, p2, p3, rl, reff):
        # find reflection of ray p0-p1 from wall p2-3
        # http://math.stackexchange.com/questions/13261/how-to-get-a-reflection-vector
        t=(p3[0]-p2[0], p3[1]-p2[1]) #tang
        n=(t[1], -t[0]) #tang
        nn=n[0]*n[0]+n[1]*n[1]
        d=(p1[0]-p0[0], p1[1]-p0[1])
        dn=2.0*(d[0]*n[0]+d[1]*n[1])/nn
        ref=(d[0]-dn*n[0], d[1]-dn*n[1])
        refl=math.hypot(ref[0], ref[1])
        #ref = (p1[0]+d[0]-dn*n[0], p1[1]+d[1]-dn*n[1])
        if refl<0.000001 or nn<0.000001 : return None
        cosa=abs((n[0]*ref[0]+n[1]*ref[1])/(math.sqrt(nn)*refl))
        ref_state=cosa<reff  # maybe makes sense to make gaussioan
        return ((p1[0]+rl*ref[0]/refl, p1[1]+rl*ref[1]/refl), cosa, ref_state)

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
        #note: sin(angle) = denom/(|s10|*|s32|)
        #where angle is a falling angle, 90 is ortho
        #or we can use (sin(angle))^2 = denom^2/(|s10|^2*|s32|^2) to avoid sqrt
        #we basically need to compare denom^2 and (|s10|^2*|s32|^2)
        # this can be used for reflection modelling, after field tests
        return intersection_point

    def Gaussian(self, mu, sigma, x):
        # calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
        if mu < 0 : mu=self.scan_max_dist
        if x < 0 : x=self.scan_max_dist
        return math.exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / math.sqrt(2.0 * math.pi * (sigma ** 2))