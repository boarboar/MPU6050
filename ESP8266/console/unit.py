import math
import timeit

class Unit:
    " Unit"
    def __init__(self, umap, pfilter):
        self.map=umap
        self.pfilter=pfilter
        #scan_a0=-90
        #scan_n=3
        #scan_d=(-scan_a0*2)/(scan_n-1)
        #scan_a0, scan_n, scan_d = -60, 6, 60
        scan_a0, scan_n, scan_d = -72, 10, 36
        self.scan_angles=[]
        self.scan_rays=[]
        for i in range(scan_n) :
            a=(scan_a0+i*scan_d)*math.pi/180.0
            self.scan_angles.append(a)
            self.scan_rays.append((math.sin(a),math.cos(a)))

        self.scan_max_dist=400
        self.bfa=30*math.pi/180 # beamform angle (30 at the moment), but more realistic is 60 deg
        pass

    def InitUnitPos(self, start):
        self.start=start
        self.isInside=False
        self.scans=[-1,-1,-1]
        self.__r_cos, self.__r_sin= (1.0, 0.0)    # unit cosine matrix, 'real'
        self.__r_x, self.__r_y = (0.0, 0.0)    # unit position, simulated
        self.__angle=0 #yaw
        self.__dist=0
        self.__move_step=0
        self.__rdist=0
        self.x_mean, self.y_mean, self.p_var, self.a_mean, self.a_var = (self.start[0], self.start[1],0,0,0) # unit localization, abs coords
        self.__l_cos, self.__l_sin= (1.0, 0.0)    # unit cosine matrix, 'localized'

    def MoveUnit(self, angle, dist, scans, x, y):
        start_time = timeit.default_timer()
        if self.__move_step == 0 : # first step
            move_rot=0
            move_dist=0
        else :
            move_rot=angle-self.__angle
            if move_rot>math.pi : move_rot=move_rot-math.pi*2
            elif move_rot<-math.pi : move_rot=math.pi*2+move_rot
            move_dist=dist-self.__dist
        self.__move_step = self.__move_step+1

        self.__r_cos=math.cos(angle)
        self.__r_sin=math.sin(angle)
        self.__angle=angle
        self.__dist=dist
        self.scans=scans
        if self.pfilter is not None :
            loc_x=self.x_mean+dist*math.sin(angle)
            loc_y=self.y_mean+dist*math.cos(angle)
            self.pfilter.updateParticles(move_dist, move_rot, scans, self.scan_angles, self.scan_max_dist, loc_x, loc_y, self.bfa)
            self.x_mean, self.y_mean, self.p_var, self.a_mean, self.a_var = self.pfilter.getMeanDistribution()
        self.__l_cos=math.cos(self.a_mean)
        self.__l_sin=math.sin(self.a_mean)

        self.__r_x, self.__r_y = x, y # simulated crd
        self.isInside=self.map.isInsideTest(x+self.start[0], y+self.start[1]) is not None

        print("Unit Mov: Rot %s Dist %s (%s, %s, %s) vs (%s, %s, %s) in %s s" %
              (str(move_rot*180.0/math.pi), move_dist,
               x+self.map.start[0], y+self.map.start[1], angle,
               self.x_mean, self.y_mean, self.a_mean,
               str(round(timeit.default_timer() - start_time, 2))))

    def GetSim(self):
        return self.__r_x+self.start[0], self.__r_y+self.start[1], self.__angle

    def UnitToMapSim(self, x, y):
        x1=x*self.__r_cos+y*self.__r_sin+self.__r_x+self.start[0]
        y1=-x*self.__r_sin+y*self.__r_cos+self.__r_y+self.start[1]
        return (x1,y1)

    def UnitToMapLoc(self, x, y):
        x1=x*self.__l_cos+y*self.__l_sin+self.x_mean
        y1=-x*self.__l_sin+y*self.__l_cos+self.y_mean
        return (x1,y1)