import json
import sys
import math
import random

from pprint import pprint

class UnitMap:
    def __init__(self, mapfile):
        self.boundRect=[sys.maxint, sys.maxint, -sys.maxint, -sys.maxint] #bounding rect

        try:
            with open(mapfile) as data_file:
                self.map = json.load(data_file)
            #pprint(__map)
            for area in self.map["AREAS"] :
                parea0=area["AT"]
                for wall in area["WALLS"] :
                    wall_crd=wall["C"]
                    self.AdjustBound(parea0[0]+wall_crd[0], parea0[1]+wall_crd[1])
                    self.AdjustBound(parea0[0]+wall_crd[2], parea0[1]+wall_crd[3])
                for obj in area["OBJECTS"] :
                    pobj0=(parea0[0]+obj["AT"][0], parea0[1]+obj["AT"][1])
                    pts=[]
                    for c in obj["CS"] :
                        p = c["C"]
                        pts.append((pobj0[0]+p[0], pobj0[1]+p[1]))
                    obj["CS_P"]=pts

            self.init_start=self.map["START"]
            print("Map loaded")
            print(self.boundRect)
        #except IOError: pass
        except : pass

    def AdjustBound(self, x, y):
        if x<self.boundRect[0] : self.boundRect[0]=x
        if y<self.boundRect[1] : self.boundRect[1]=y
        if x>self.boundRect[2] : self.boundRect[2]=x
        if y>self.boundRect[3] : self.boundRect[3]=y

    def At(self, cell):
        # cell status : 0-space, 1-occupied, 2-variable
        status=0

        for v in cell :
            if not self.isInsideTest(v[0], v[1]) :
                status=1
                break
        if status != 0 : return status

        # bug - it's possible that all thre points are inside, but internal wall is inside the cell...
        # just for now - test a center point
        if not self.isInsideTest((cell[0][0]+cell[-2][0])/2, (cell[0][1]+cell[2][1])/2) : return 1

        for area in self.map["AREAS"] :
            parea0=area["AT"]
            try:
                for obj in area["OBJECTS"] :
                    """
                    pobj0=(parea0[0]+obj['AT'][0], parea0[1]+obj['AT'][1])
                    ovs=[]
                    for c in obj['CS'] :
                        op=c['C']
                        ovs.append((pobj0[0]+op[0], pobj0[1]+op[1]))
                    """
                    status=self.polyIntersects(cell, obj['CS_P'])
                    if status!=0 : break
            except KeyError :
                print('Some obj attr missing')
                pass

            if status!=0 : break

        return status

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

    def getIntersectionMapRefl(self, p0, p1, scan_max_dist):

        intrs0, ref=self.getIntersectionMap(p0, p1, True, scan_max_dist)
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
                intrs1, ref=self.getIntersectionMap(intrs0, pr, False, scan_max_dist)
                intrs=intrs1
        return (intrs0, pr, intrs1, refState, intrs)

    def getIntersectionMap(self, p0, p1, findRefl, scan_max_dist):
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
                print('Some obj attr missing')
                pass


        #return intrs, ref

        if intrs!=None and findRefl :
            # find reflection vector
            rl=scan_max_dist-math.sqrt(dist2)
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

    def polyIntersects(self,  vs0, vs1 ) :
        isect=None
        inside = False
        v00=vs0[-1]
        for v0 in vs0 :
            v10=vs1[-1]
            for v1 in vs1 :
                isect=self.find_intersection(v00, v0, v10, v1)
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
            isect=self.intersectHor(p[1], v0[0], v0[1], v[0], v[1])
            if isect!=None :
                if isect<p[0] : left=left+1
                else : right=right+1
            v0=v
        if left%2==1 and right%2==1 :
            return True
        return False
