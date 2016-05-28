import math
import random
import copy

class Particle:
    def __init__(self, x=150, y=200, a=0.0, w=1.0, id=0):
        self.x, self.y = (x, y)
        self.a=a
        self.w=w
        self.id=id

    def move_d(self, dist, da):
        self.a=self.a+da
        if self.a>math.pi : self.a=self.a-math.pi*2
        elif self.a<-math.pi : self.a=math.pi*2+self.a
        self.x=self.x+dist*math.sin(self.a)
        self.y=self.y+dist*math.cos(self.a)

    def __repr__(self):
        return 'P_%s <(%s,%s) A %s W %s >' % (str(self.id), str(self.x), str(self.y), str(self.a*180.0/math.pi), str(self.w))

class PFilter:
    " Particle filter"
    def __init__(self, umap):
        self.map=umap
        self.particles=[]
        self.fwd_noise=5
        self.rot_noise=0.5
        self.sense_noise=20

    def InitParticles(self):
        N_D=20
        W=1.0/(N_D*N_D)
        LOC_VAR=150
        #ANG_VAR=math.pi/2
        ANG_VAR=math.pi
        self.particles = [] #clean
        for i in range(N_D) :
            for j in range(N_D) :
                a=(random.random()-0.5)*ANG_VAR*2
                x=(random.random()-0.5)*LOC_VAR+self.map.start[0]
                y=(random.random()-0.5)*LOC_VAR+self.map.start[1]
                self.particles.append(Particle(a=a, x=x, y=y, id=i*N_D+j, w=W))

    def updateParticles(self, mov, rot, scans, scan_angles, scan_max_dist):
        if len(self.particles)==0 : return
        for p in self.particles :
            p.move_d(mov+random.gauss(0, self.fwd_noise), rot+random.gauss(0, self.rot_noise))
            if self.map.isInsideTest(p.x, p.y) :
                self.updateParticleProbabilities(p, scans, scan_angles, scan_max_dist)
            else : p.w=0.0
        #print self.particles
        mw=max(p.w for p in self.particles)
#        if self.__move < 5 or self.__move%5==0 :
        if True :
            #resmple, use algorithm from udacity
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
        #self.x_mean, self.y_mean, self.p_var, self.a_mean, self.a_var = self.getMeanDistribution()

    def getMeanDistribution(self):
        # mean distrubution of coords and angles
        # weighted average of circular data - see
        # http://stackoverflow.com/questions/491738/how-do-you-calculate-the-average-of-a-set-of-circular-data
        x,y,var=0,0,0
        #a,vara=0, 0
        x_a,y_a= 0,0
        a_a,var_a_a=0,0
        for p in self.particles :
            x+=p.x*p.w
            y+=p.y*p.w
            #a+=p.a*p.w ######### BUG in area of 180/-180 angle!!!!!!!!!!! Can not sum circular values
            x_a+=math.sin(p.a)*p.w
            y_a+=math.cos(p.a)*p.w

        a_a=math.atan2(y_a, x_a)
        a_a=math.pi*0.5-a_a
        if a_a < -math.pi :
            a_a += 2*math.pi
        if a_a > math.pi :
            a_a -= 2*math.pi

        for p in self.particles :
            ex=p.x-x
            ey=p.y-y
            var += (ex*ex+ey*ey)*p.w
            #ea=p.a-a
            #vara+=ea*ea*p.w
            e_a_a=p.a-a_a
            if e_a_a < -math.pi :
                e_a_a += 2*math.pi
            if e_a_a > math.pi :
                e_a_a -= 2*math.pi
            var_a_a+=e_a_a*e_a_a*p.w


        var= math.sqrt(var)
        #vara= math.sqrt(vara)
        var_a_a= math.sqrt(var_a_a)
        #print("Variance %s" % str(round(var,2)))

        #print("A/CA %s(%s) %s(%s)" % (round(a,2),round(vara,2),round(a_a,2), round(var_a_a,2),))

        #return (x, y, var, a, vara)
        return (x, y, var, a_a, var_a_a)

    def updateParticleProbabilities(self, p, meas, scan_angles, scan_max_dist):
        scan_dist=[]
        prob = 1.0;
        p0=(p.x, p.y)
        for i in range(len(scan_angles)) :
            a=scan_angles[i]
            p1=(p.x+math.sin(p.a+a)*scan_max_dist, p.y+math.cos(p.a+a)*scan_max_dist)
            intrs0, pr, intrs1, refstate, intrs = self.map.getIntersectionMapRefl(p0, p1, scan_max_dist)
            if intrs==None : dist2 = -1
            else :
                dist2=math.sqrt((intrs[0]-p0[0])*(intrs[0]-p0[0])+(intrs[1]-p0[1])*(intrs[1]-p0[1]))
            scan_dist.append(dist2)
            prob*=self.Gaussian(dist2, self.sense_noise, meas[i], scan_max_dist)
        p.w=prob
        #print(scan_dist)

    """
    def getParticleRays(self, p): #just for visual debugging
        rays=[]
        p0=(p.x, p.y)
        for a in self.scan_angles :
            p1=(p.x+math.sin(p.a+a)*self.scan_max_dist, p.y+math.cos(p.a+a)*self.scan_max_dist)
            rays.append((p0, p1))
        return rays
    """
    """
    def getParticleIntersects(self, p): #just for visual debugging
        intersects=[]
        p0=(p.x, p.y)
        for a in self.scan_angles :
            p1=(p.x+math.sin(p.a+a)*self.scan_max_dist, p.y+math.cos(p.a+a)*self.scan_max_dist)
            intrs, ref = self.getIntersectionMap(p0, p1)
            intersects.append(intrs)
        return intersects
    """

    def Gaussian(self, mu, sigma, x, scan_max_dist):
        # calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
        if mu < 0 : mu=scan_max_dist
        if x < 0 : x=scan_max_dist
        return math.exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / math.sqrt(2.0 * math.pi * (sigma ** 2))