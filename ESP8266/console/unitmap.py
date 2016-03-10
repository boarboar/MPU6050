import json
import sys
import math

from pprint import pprint

class UnitMap:
    def __init__(self, mapfile):
        self.xu0, self.yu0 = (0, 0)    # unit base (start point)
        self.__r_cos, self.__r_sin= (1.0, 0.0)    # unit cosine matrix
        self.__r_x, self.__r_y = (0, 0)    # unit position
        self.boundRect=[sys.maxint, sys.maxint, -sys.maxint, -sys.maxint] #bounding rect
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

    def SetUnitPos(self, x, y, angle):
        self.__r_cos=math.cos(angle)
        self.__r_sin=math.sin(angle)
        self.__r_x=x
        self.__r_y=y

    def UnitToMap(self, x, y):
        x1=x*self.__r_cos+y*self.__r_sin+self.__r_x+self.xu0
        y1=-x*self.__r_sin+y*self.__r_cos+self.__r_y+self.yu0
        return (x1,y1)

    def adjustBound(self, x, y):
        if x<self.boundRect[0] : self.boundRect[0]=x
        if y<self.boundRect[1] : self.boundRect[1]=y
        if x>self.boundRect[2] : self.boundRect[2]=x
        if y>self.boundRect[3] : self.boundRect[3]=y

    def isInside(self, x, y):
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
                        #intrs.append(isect)
                        intrs=isect
                        dist2=d2
        print (intrs)
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