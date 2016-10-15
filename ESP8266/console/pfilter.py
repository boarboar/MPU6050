import math
import random
import copy
import timeit
from operator import attrgetter
import bisect

class WeightedDistribution(object):
    def __init__(self, state):
        accum = 0.0
        self.state = [p for p in state if p.w > 0]
        self.distribution = []
        for x in self.state:
            accum += x.w
            self.distribution.append(accum)

    def pick(self):
        try:
            return self.state[bisect.bisect_left(self.distribution, random.uniform(0, 1))]
        except IndexError:
            # Happens when all particles are improbable w=0
            return None

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
        #self.fwd_noise=5
        self.fwd_noise=20
        self.rot_noise=0.1
        #self.rot_noise=0.05
        #self.sense_noise=20
        self.sense_noise=200

        self.gauss_denom=math.sqrt(2.0 * math.pi * (self.sense_noise ** 2))
        self.gauss_exp_denom=(self.sense_noise ** 2) * 2.0

    def InitParticles(self):
        #N_D=20
        N_D=14
        W=1.0/(N_D*N_D)
        #LOC_VAR=150
        LOC_VAR=100
        #ANG_VAR=math.pi/2
        ANG_VAR=math.pi/4
        #ANG_VAR=math.pi
        self.particles = [] #clean
        for i in range(N_D) :
            for j in range(N_D) :
                a=(random.random()-0.5)*ANG_VAR*2
                x=(random.random()-0.5)*LOC_VAR+self.map.init_start[0]
                y=(random.random()-0.5)*LOC_VAR+self.map.init_start[1]
                self.particles.append(Particle(a=a, x=x, y=y, id=i*N_D+j, w=W))

    def printParticles(self, header=None):
        if header is not None: print(header)
        psorted=sorted(self.particles, key=attrgetter('w'), reverse=True)
        for p in psorted :
            print("C=(%s,%s), A=%s, W=%s" % (round(p.x,0), round(p.y,0), round(p.a,0), p.w) )

    def updateParticles(self, mov, rot, scans, scan_angles, scan_max_dist, loc_x, loc_y):
        if len(self.particles)==0 : return

        scan_angles_cos=[]
        scan_angles_sin=[]
        for a in scan_angles :
            scan_angles_cos.append(math.cos(a))
            scan_angles_sin.append(math.sin(a))


        #self.printParticles("Before")

        #print (self.map.getSortedWalls((0,0)))

        sorted_walls_base=self.map.getSortedWalls((loc_x, loc_y), scan_max_dist)


        start_time = timeit.default_timer()
        for p in self.particles :
            #sorted_walls_base=self.map.getSortedWalls((loc_x, loc_y), scan_max_dist)
            p.move_d(mov+random.gauss(0, self.fwd_noise), rot+random.gauss(0, self.rot_noise))
            if self.map.isInsideTest(p.x, p.y) is not None :
                sorted_walls=self.map.getReSortedWalls(sorted_walls_base, (p.x, p.y), scan_max_dist)
                self.updateParticleProbabilities(p, scans, scan_angles, scan_max_dist, sorted_walls)
                ############ just for tests, replaced with old version
                #self.updateParticleProbabilities1(p, scans, scan_angles_cos, scan_angles_sin, scan_max_dist, sorted_walls)
            else : p.w=0.0

        t=timeit.default_timer() - start_time
        print ('Particle update (%s) in %s s, %s per particle' %
               (len(self.particles), round(t, 2), round(t/len(self.particles), 4))
               )

        wsum=sum(p.w for p in self.particles)
        if wsum>0 :
            for p in self.particles :
                p.w = p.w/wsum

        #self.printParticles("After W update")

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


        """
        dist = WeightedDistribution(self.particles)
        p3 = []
        for _ in self.particles:
            p = dist.pick()
            if p is None:  # No pick b/c all totally improbable
                pass
            else:
                p3.append(copy.copy(p))
        self.particles = p3
        """

        #self.printParticles("After Rsample")

        #normalize
        wsum=sum(p.w for p in self.particles)
        if wsum>0 :
            for p in self.particles :
                p.w = p.w/wsum

        #self.printParticles("Normed")
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


    def updateParticleProbabilities1(self, p, meas, scan_angles_cos, scan_angles_sin, scan_max_dist, sorted_walls):
        #scan_dist=[]
        prob = 1.0
        p0=(p.x, p.y)
        cosp=math.cos(p.a)
        sinp=math.sin(p.a)
        for i in range(len(scan_angles_cos)) :
            cosa=scan_angles_cos[i]
            sina=scan_angles_sin[i]
            cospa=cosa*cosp-sina*sinp
            sinpa=cosa*sinp+sina*cosp
            p1=(p.x+sinpa*scan_max_dist, p.y+cospa*scan_max_dist)
            intrs0, pr, intrs1, refstate, intrs, cosa2 = self.map.getIntersectionMapRefl(p0, p1, scan_max_dist, sorted_walls)
            if intrs==None : dist2 = -1
            else :
                dist2=math.sqrt((intrs[0]-p0[0])*(intrs[0]-p0[0])+(intrs[1]-p0[1])*(intrs[1]-p0[1]))
            #scan_dist.append(dist2)
            prob*=self.Gaussian1(dist2, meas[i], scan_max_dist)
        p.w=prob
        #print(scan_dist)

    def updateParticleProbabilities(self, p, meas, scan_angles, scan_max_dist, sorted_walls):
        #scan_dist=[]
        prob = 1.0
        p0=(p.x, p.y)
        for i in range(len(scan_angles)) :
            a=scan_angles[i]
            p1=(p.x+math.sin(p.a+a)*scan_max_dist, p.y+math.cos(p.a+a)*scan_max_dist)
            intrs0, pr, intrs1, refstate, intrs, cosa2 = self.map.getIntersectionMapRefl(p0, p1, scan_max_dist, sorted_walls)
            if intrs==None : dist2 = -1
            else :
                dist2=math.sqrt((intrs[0]-p0[0])*(intrs[0]-p0[0])+(intrs[1]-p0[1])*(intrs[1]-p0[1]))
            #scan_dist.append(dist2)

            prob*=self.Gaussian1(dist2, meas[i], scan_max_dist)
            #prob*=self.Gaussian(dist2, self.sense_noise, meas[i], scan_max_dist)

        p.w=prob
        #p.w=math.log10(prob)
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
    """
    def Gaussian(self, mu, sigma, x, scan_max_dist):
        # calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
        if mu < 0 : mu=scan_max_dist
        if x < 0 : x=scan_max_dist
        return math.exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / math.sqrt(2.0 * math.pi * (sigma ** 2))
    """

    def Gaussian1(self, mu, x, scan_max_dist):
        # calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
        if mu < 0 : mu=scan_max_dist
        if x < 0 : x=scan_max_dist
        #dist=abs(mu-x)
        #if x>scan_max_dist/2 : dist=dist/2
        #if x>1 : dist=dist/x
        dist2=(mu-x)**2
        #if x>1 : dist2*=(x+mu)/2
        #if x>1 : dist2/=(x+mu)/2
        return math.exp(-dist2/ self.gauss_exp_denom) / self.gauss_denom