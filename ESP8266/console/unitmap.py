import json
import sys
import math
import random
from operator import itemgetter
import timeit
import geometry
import numpy as np

from pprint import pprint

# wall struct
# 0: p0
# 1: p1
# 2: S
# 3: refl
# 4: density
# %: dist

# New wall struct
# 0: dist
# 1: Stat
# 2: p0
# 3: p1
# 4: refl
# 5: density

class Wall(object):
    def __init__(self, p0, p1, S, refl, density, dist):
        self.p0=p0
        self.p1 = p1
        self.S = S
        self.refl=refl
        self.density=density
        self.dist=dist

class UnitMap:
    GRID_SZ=15 #HxW cm
    GRID_DELTA=4 #cm

    #GRID_SZ = 8  # HxW cm
    #GRID_DELTA = 8  # cm

    def __init__(self, mapfile):
        self.boundRect=[sys.maxint, sys.maxint, -sys.maxint, -sys.maxint] #bounding rect
        self.resetCounters()
        try:
            with open(mapfile) as data_file:
                self.map = json.load(data_file)
            #pprint(__map)
            area_id=0
            walls=[]
            for area in self.map["AREAS"] :
                area_id+=1
                parea0=area["AT"]
                area["ID"]=area_id
                for wall in area["WALLS"] :
                    parea0=area["AT"]
                    wall_crd=wall["C"]
                    self.AdjustBound(parea0[0]+wall_crd[0], parea0[1]+wall_crd[1])
                    self.AdjustBound(parea0[0]+wall_crd[2], parea0[1]+wall_crd[3])
                    opened=0
                    if 'S' in wall : opened=wall["S"]
                    walls.append((
                        (parea0[0]+wall_crd[0], parea0[1]+wall_crd[1]), (parea0[0]+wall_crd[2], parea0[1]+wall_crd[3]),
                        opened, area["WALLSREFL"], 1
                    ))
                for obj in area["OBJECTS"] :
                    pobj0=(parea0[0]+obj["AT"][0], parea0[1]+obj["AT"][1])
                    free_pos=0 #0-fixed; 2-freepos
                    if 'F' in obj : free_pos=obj['F']
                    density=1
                    if 'DENSITY' in obj : density=obj['DENSITY']
                    obj_walls=[]
                    crd_p=[]
                    obj["CS_P"]=crd_p
                    if 'CS' in obj :
                        for c in obj["CS"] :
                            p = c["C"]
                            crd_p.append((pobj0[0]+p[0], pobj0[1]+p[1]))

                        """
                        #if len(crd_p)<2 or density < 0.1 : continue
                        obj_walls=[]
                        op0=crd_p[-1]
                        for op in crd_p :
                            walls.append(( op0, op, free_pos, 0, density))
                            obj_walls.append(( op0, op))
                            op0=op
                        """

                        p=obj['CS'][-1]['C']
                        op0=(pobj0[0]+p[0], pobj0[1]+p[1])
                        for c in obj['CS'] :
                            p = c["C"]
                            op=(pobj0[0]+p[0], pobj0[1]+p[1])
                            walls.append(( op0, op, free_pos, 0, density))
                            obj_walls.append(( op0, op, free_pos, 0, density))
                            op0=op


                    elif 'CCS' in obj :
                         for sobj in obj["CCS"] :
                            psobj0=pobj0
                            if 'AT' in sobj : psobj0=(pobj0[0]+sobj['AT'][0], pobj0[1]+sobj['AT'][1])
                            sdensity=1
                            if 'DENSITY' in sobj : sdensity=sobj['DENSITY']
                            p=sobj['CS'][-1]['C']
                            op0=(psobj0[0]+p[0], psobj0[1]+p[1])
                            for c in sobj['CS'] :
                                p = c["C"]
                                op=(psobj0[0]+p[0], psobj0[1]+p[1])
                                walls.append(( op0, op, free_pos, 0, sdensity))
                                obj_walls.append(( op0, op, free_pos, 0, sdensity))
                                op0=op
                                crd_p.append((op[0], op[1]))

                    obj["WALLS"]=obj_walls



            self.map["WALLS"]=walls

            self.init_start=self.map["START"]

            print("Map loaded, there are %s walls" % (len(walls)))
            print(self.boundRect)
        except IOError: print("IO Error")
        #except : pass
        w=self.boundRect[2]-self.boundRect[0]
        h=self.boundRect[3]-self.boundRect[1]
        nx=w/self.GRID_SZ+1
        ny=h/self.GRID_SZ+1
        x0=self.boundRect[0]
        y0=self.boundRect[1]
        print('init grid NR=%s NC=%s...' % (ny, nx))
        start_time = timeit.default_timer()
        cells_unused = 0
        cells_avail = 0
        self.grid = [[[x0+col*self.GRID_SZ,y0+row*self.GRID_SZ, None, None, None] for col in range(nx)] for row in range(ny)]
        for row in self.grid:
            for cell in row:
                x, y =cell[0], cell[1]
                area=[(x-self.GRID_DELTA,y-self.GRID_DELTA),
                      (x+self.GRID_SZ+self.GRID_DELTA, y-self.GRID_DELTA),
                      (x+self.GRID_SZ+self.GRID_DELTA, y+self.GRID_SZ+self.GRID_DELTA),
                      (x-self.GRID_DELTA, y+self.GRID_SZ+self.GRID_DELTA)]
                cell[2]=self.AtSlow(area)
                if cell[2] == 1:
                    cell[3] = []  # unused/ occupied
                    cells_unused += 1
                else:
                    cell[3] = self._getSortedWalls((x+self.GRID_SZ/2, y+self.GRID_SZ/2), 500)
                    cells_avail += 1
                cell[4]=0   # hit count
        print('grid inited in %s s, unused %s, avail %s' %
              (round(timeit.default_timer() - start_time, 2), cells_unused, cells_avail))

        self.i_fun = geometry.c_find_intersection

        count_walls = 0
        meaningless = 0
        start_time = timeit.default_timer()
        for row in self.grid:
            for cell in row:
                walls = cell[3]
                for w in walls:
                    count_walls += 1
                    meaningless += w[0]
                    #meaningless += w.dist
        print('traversed %s walls, ml=%s,  in %s' %
              (count_walls, meaningless, round(timeit.default_timer() - start_time, 2)))

        self.counter_t_upd = 0

    def resetCounters(self) :
        #self.counter_map_sortwalls = 0
        #self.counter_map_refl = 0
        #self.counter_map_refl_refl = 0
        self.counter_t_upd = 0

    def getCounters(self):
        return self.counter_t_upd

    def getSortedWalls(self, p):
        cell0=self.grid[0][0]
        r=int((p[1]-cell0[1])/self.GRID_SZ)
        c=int((p[0]-cell0[0])/self.GRID_SZ)
        if r >= 0 and r < len(self.grid) and c >=0 and c < len(self.grid[r]):
            return self.grid[r][c][3]
        else:
            return []

    def _getSortedWalls(self, p, scan_max_dist):
        #self.counter_map_sortwalls = self.counter_map_sortwalls +1
        wall_dist=[]
        walls_all=self.map["WALLS"]
        maxdist2=scan_max_dist*scan_max_dist
        for walls in walls_all :
            p0=walls[0]
            p1=walls[1]
            d0=(p0[0]-p[0])*(p0[0]-p[0])+(p0[1]-p[1])*(p0[1]-p[1])
            d1=(p1[0]-p[0])*(p1[0]-p[0])+(p1[1]-p[1])*(p1[1]-p[1])
            d=d0
            if d1<d0 : d=d1
            if d<maxdist2 and walls[4]>0.1 :
                #wall_dist.append([p0, p1, walls[2], walls[3], walls[4], d])
                #wall_dist.append([d, walls[2], p0, p1, walls[3], walls[4]])
                wall_dist.append([d, walls[2], p0, p1])

        return sorted(wall_dist, key=itemgetter(0))

        #return sorted(wall_dist, key=itemgetter(5))

        #wall_dist = sorted(wall_dist, key=itemgetter(5))
        #return np.array([Wall(w[0], w[1], w[2], w[3], w[4], w[5]) for w in wall_dist])

    def getReSortedWalls(self, walls0, p, scan_max_dist):
        wall_dist=[]
        maxdist2=scan_max_dist*scan_max_dist
        for walls in walls0 :
            p0=walls[0]
            p1=walls[1]
            d0=(p0[0]-p[0])*(p0[0]-p[0])+(p0[1]-p[1])*(p0[1]-p[1])
            d1=(p1[0]-p[0])*(p1[0]-p[0])+(p1[1]-p[1])*(p1[1]-p[1])
            d=d0
            if d1<d0 : d=d1
            if d<maxdist2 :
                wall_dist.append((p0, p1, walls[2], walls[3], walls[4], d))

        return sorted(wall_dist, key=itemgetter(5))

    def AdjustBound(self, x, y):
        if x<self.boundRect[0] : self.boundRect[0]=x
        if y<self.boundRect[1] : self.boundRect[1]=y
        if x>self.boundRect[2] : self.boundRect[2]=x
        if y>self.boundRect[3] : self.boundRect[3]=y

    def UnitMoved(self, unit):
        if unit.scans is not None:
            i = 0
            cell0 = self.grid[0][0]
            for ray in unit.scan_rays:
                idist = unit.scans[i]
                i = i+1
                if idist > 0:
                    p = unit.UnitToMapLoc(ray[0] * idist, ray[1] * idist)
                    r = int((p[1] - cell0[1]) / self.GRID_SZ)
                    c = int((p[0] - cell0[0]) / self.GRID_SZ)
                    #print c, r, p
                    if r >= 0 and r < len(self.grid) and c >= 0 and c < len(self.grid[r]):
                        self.grid[r][c][4] = self.grid[r][c][4]+1


    def AtSlow(self, cell):
        # cell status : 0-space, 1-occupied/unusable, 2-variable
        status=0

        for v in cell :
            if self.isInsideTestSlow(v[0], v[1]) is None :
                status=1
                break
        if status != 0 : return status

        # bug - it's possible that all the points are inside, but internal wall is inside the cell...
        # just for now - test a center point
        if self.isInsideTestSlow((cell[0][0]+cell[-2][0])/2, (cell[0][1]+cell[2][1])/2) is None : return 1

        for area in self.map["AREAS"] :
            parea0=area["AT"]
            try:
                if "OBJECTS" in area :
                    for obj in area["OBJECTS"] :
                        if 'CS_P' in obj :
                            status=self.polyIntersects(cell, obj['CS_P'])
                            if status!=0 : break
            except KeyError :
                print('AT: Some obj attr missing')
                pass

            if status!=0 : break

        return status

    def isInsideTestSlow(self, x, y):
        for area in self.map["AREAS"] :
            left, right = (0, 0)
            for wall in area["WALLS"] :
                parea0=area["AT"]
                wall_crd=wall["C"]
                isect=self.___intersectHor(y, parea0[0]+wall_crd[0], parea0[1]+wall_crd[1],
                                        parea0[0]+wall_crd[2], parea0[1]+wall_crd[3])
                if isect!=None :
                    if isect<x : left=left+1
                    else : right=right+1
                """

                isect=geometry.c_find_intersection((-10000, y), (10000, y), (parea0[0]+wall_crd[0], parea0[1]+wall_crd[1]),
                                                   (parea0[0]+wall_crd[2], parea0[1]+wall_crd[3]))

                if isect!=None :
                    if isect[0]<x : left=left+1
                    else : right=right+1
                """

            if left%2==1 and right%2==1 : return area
        return None

    def ___At(self, cell):
        # cell status : 0-space, 1-occupied/unusable, 2-variable
        status=0

        for v in cell :
            if self.isInsideTest(v[0], v[1]) is None :
                status=1
                break
        if status != 0 : return status

        return status

        # bug - it's possible that all the points are inside, but internal wall is inside the cell...
        # just for now - test a center point
        if self.isInsideTest((cell[0][0]+cell[-2][0])/2, (cell[0][1]+cell[2][1])/2) is None : return 1

        for area in self.map["AREAS"] :
            parea0=area["AT"]
            try:
                if "OBJECTS" in area :
                    for obj in area["OBJECTS"] :
                        if 'CS_P' in obj :
                            status=self.polyIntersects(cell, obj['CS_P'])
                            if status!=0 : break
            except KeyError :
                print('AT: Some obj attr missing')
                pass

            if status!=0 : break

        return status

    def isInsideTest(self, x, y):
        for area in self.map["AREAS"] :
            left, right = (0, 0)
            for wall in area["WALLS"] :
                parea0=area["AT"]
                wall_crd=wall["C"]

                isect=geometry.c_find_intersection((-10000, y), (10000, y), (parea0[0]+wall_crd[0], parea0[1]+wall_crd[1]),
                                                   (parea0[0]+wall_crd[2], parea0[1]+wall_crd[3]))
                if isect!=None :
                    if isect[0]<x : left=left+1
                    else : right=right+1

            if left%2==1 and right%2==1 : return area
        return None

    def isInsideTestFast(self, x, y):
        cell0 = self.grid[0][0]
        r = int((y - cell0[1]) / self.GRID_SZ)
        c = int((x - cell0[0]) / self.GRID_SZ)
        if r >= 0 and r < len(self.grid) and c >= 0 and c < len(self.grid[r]):
            # print("[%s, %s] (%s,%s) %s" % (x, y, c, r, self.grid[r][c][2]))
            return self.grid[r][c][2] != 1
        else:
            # print("[%s, %s] (%s,%s) %s" % (x, y, c, r, '#'))
            return False
    """
    def getIntersectionMapClean(self, p0, p1, sorted_walls):
        intrs0, dumped, dist = self.getIntersectionMapSimple(p0, p1, sorted_walls)
        d = None
        if intrs0 is not None:
            if not dumped:
                d = dist
        return d
    """
    def getIntersectionMapClean(self, p0, p1, sorted_walls):
        # line p0->p1 in absolute map coords (world)
        intrs = None
        dist2 = 0  #square dist
        # i_fun=self.find_intersection
        i_fun = geometry.c_find_intersection
        rnd = random.random
        for walls in sorted_walls:
            #if (dist2 > 0) and (walls[5] > dist2):
            if (dist2 > 0) and (walls[0] > dist2):
                break
            isect = None
            #movable = walls[2]
            movable = walls[1]
            if movable == 0 or (movable == 2 and rnd() > 0.5):
                #p2 = walls[0]
                #p3 = walls[1]
                p2 = walls[2]
                p3 = walls[3]
                isect = i_fun(p0, p1, p2, p3)

            if isect is not None:
                #d2 = (isect[0] - p0[0]) * (isect[0] - p0[0]) + (isect[1] - p0[1]) * (isect[1] - p0[1])
                dx, dy = isect[0] - p0[0], isect[1] - p0[1]
                d2 = dx*dx+dy*dy
                if (intrs is None or d2 < dist2) and d2 > 0.001:
                    intrs = isect
                    dist2 = d2

        if intrs is not None:
            return math.sqrt(dist2)
        else:
            return None

    def getIntersectionMapRefl(self, p0, p1, scan_max_dist, sorted_walls, for_draw=False):
        #intrs0, ref, dumped, dist =self.getIntersectionMap1(p0, p1, True, scan_max_dist, sorted_walls, for_draw)
        intrs0, ref, dumped, dist =self.getIntersectionMap1(p0, p1, False, scan_max_dist, sorted_walls, for_draw)
        """
        if for_draw and dumped :
            print('Dumped')
            print(intrs0)
        """
        #self.counter_map_refl = self.counter_map_refl+1
        refState = False
        pr = None
        intrs1 = None
        intrs = None
        cosa2= None
        if intrs0 != None:
            if ref != None :
                pr, cosa2, refState = ref


            if not dumped:
                intrs=intrs0

                """
                if refState:
                    # secondary intersect if any
                    # reduce max dist due to signal loss
                    max_dist_refl=(scan_max_dist-dist)/4
                    if max_dist_refl > 5 :
                        if dist>10 :
                            refl_sorted_walls=self.getSortedWalls(intrs, max_dist_refl)
                        else :
                            refl_sorted_walls = sorted_walls
                        intrs1, ref, dumped, dist_ref=self.getIntersectionMap1(intrs0, pr, False, max_dist_refl, refl_sorted_walls)
                        intrs=intrs1
                        dist=dist+dist_ref
                        self.counter_map_refl_refl = self.counter_map_refl_refl+1
                """
        return (intrs0, pr, intrs1, refState, intrs, cosa2, dist)


    def getIntersectionMap1(self, p0, p1, findRefl, scan_max_dist, sorted_walls, for_draw=False):
        # line p0->p1 in absolute map coords (world)
        intrs = None
        ref=None
        dist2=0
        reff=0
        density=1
        dumped=False
        iwall=None
        for walls in sorted_walls :
            if dist2 > 0 and walls[5] > dist2 : break
            isect=None
            p2=walls[0]
            p3=walls[1]
            movable=walls[2]
            if movable==0 or (movable==2 and random.random()>0.5):
                #isect=self.find_intersection(p0, p1, p2, p3)
                isect=geometry.c_find_intersection(p0, p1, p2, p3)
            if isect!=None :
                d2 = (isect[0]-p0[0])*(isect[0]-p0[0])+(isect[1]-p0[1])*(isect[1]-p0[1])
                if (intrs==None or d2<dist2) and d2>0.01:
                    intrs=isect
                    dist2=d2
                    wsect=(p2, p3)
                    reff=walls[3]
                    density=walls[4]

        dist=math.sqrt(dist2)
        if intrs!=None and findRefl :
            if density<0.99:
                ref=self.getReflection(p0, intrs, wsect[0], wsect[1], dist, density)
                if ref is not None :
                    pr, cc, refState = ref
                    if refState :
                        dumped=True
                        #print()
                        #intrs=None #- if to make transparent

            # find reflection vector
            #elif (scan_max_dist-50)*(scan_max_dist-50) > dist2 :
            elif (scan_max_dist-50) > dist :
                ref=self.getReflection(p0, intrs, wsect[0], wsect[1], dist, reff)

        return intrs, ref, dumped, dist

    def __getIntersectionMap(self, p0, p1, findRefl, scan_max_dist):
        """
        OBSOLETE
        """
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
                    crd_p=obj['CS_P']
                    if len(crd_p)<2 or obj['DENSITY'] < 0.1 : continue

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

                    free_pos=0 #0-fixed; 2-freepos
                    if 'F' in obj : free_pos=obj['F']
                    op0=crd_p[-1]
                    for op in crd_p :
                        isect=None
                        if free_pos==0 or (free_pos==2 and random.random()>0.5) :
                            isect=self.find_intersection(p0, p1, op0, op)
                        if isect!=None :
                            d2 = (isect[0]-p0[0])*(isect[0]-p0[0])+(isect[1]-p0[1])*(isect[1]-p0[1])
                            if intrs==None or d2<dist2 :
                                intrs=isect
                                dist2=d2
                                wsect=(op0, op)
                                reff=0
                        op0=op

            except KeyError :
                print('GIM: Some obj attr missing')
                pass


        #return intrs, ref

        if intrs!=None and findRefl :
            # find reflection vector
            rl=scan_max_dist-math.sqrt(dist2)
            #if rl>0 :
            if rl<=0 : rl=10
            ref=self.getReflection(p0, intrs, wsect[0], wsect[1], rl, reff)

        return intrs, ref

    def ___intersectHor(self, y, x0, y0, x1, y1):
        if y0==y1 : #with hor line
            if y!=y0 : return None # no intersect
            else : return None # SPECIAL CASE :: ON LINE (TODO)
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
        #cosa=abs((n[0]*ref[0]+n[1]*ref[1])/(math.sqrt(nn)*refl))
        #ref_state=cosa<reff  # maybe makes sense to make gaussioan
        prod=(n[0]*ref[0]+n[1]*ref[1])
        cosa2=prod*prod/(nn*refl*refl)
        ref_state=cosa2<reff*reff
        #return ((p1[0]+rl*ref[0]/refl, p1[1]+rl*ref[1]/refl), cosa, ref_state)
        return ((p1[0]+rl*ref[0]/refl, p1[1]+rl*ref[1]/refl), cosa2, ref_state)

    def __find_intersection(self,  p0, p1, p2, p3 ) :
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
        t = float(t_numer) / denom
        #print(p0[0], p0[1], s10_x, s10_y, denom, s_numer, t_numer, t)
        #intersection_point = ( int(p0[0] + (t * s10_x)), int(p0[1] + (t * s10_y)) )
        intersection_point = ( p0[0] + (t * s10_x), p0[1] + (t * s10_y) )
        #note: sin(angle) = denom/(|s10|*|s32|)
        #where angle is a falling angle, 90 is ortho
        #or we can use (sin(angle))^2 = denom^2/(|s10|^2*|s32|^2) to avoid sqrt
        #we basically need to compare denom^2 and (|s10|^2*|s32|^2)
        # this can be used for reflection modelling, after field tests
        return intersection_point

    def polyIntersects(self,  vs0, vs1 ) :
        isect=None
        inside = False
        v00=vs0[-1]
        for v0 in vs0 :
            v10=vs1[-1]
            for v1 in vs1 :
                #isect=self.find_intersection(v00, v0, v10, v1)
                isect=geometry.c_find_intersection(v00, v0, v10, v1)
                if isect!=None : break
                v10=v1
            if isect!=None : break
            v00=v0

        if isect is None :
            #test if one point of area1 inside of area2
            p=((vs1[0][0]+vs1[-1][0])/2,(vs1[0][1]+vs1[-1][1])/2)
            inside=self.polyInside(vs0, p)
            if not inside :
                p=((vs0[0][0]+vs0[-1][0])/2,(vs0[0][1]+vs0[-1][1])/2)
                inside=self.polyInside(vs1, p)

        if isect!=None or inside : return 1
        return  0

    def polyInside(self,  vs, p ) :
        left, right = (0, 0)
        v0=vs[-1]
        for v in vs :
            """
            isect=self.intersectHor(p[1], v0[0], v0[1], v[0], v[1])
            if isect!=None :
                if isect<p[0] : left=left+1
                else : right=right+1
            """

            isect=geometry.c_find_intersection((-10000, p[1]), (10000, p[1]), v0, v)
            if isect!=None :
                if isect[0]<p[0] : left=left+1
                else : right=right+1

            v0=v
        if left%2==1 and right%2==1 :
            return True
        return False
