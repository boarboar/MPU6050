import json
import sys
from pprint import pprint

class UnitMap:
    def __init__(self, mapfile):
        self.xu0, self.yu0 = (0, 0)    # unit base (start point)
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
                if isect[0]==True :
                    if isect[1]<x : left=left+1
                    else : right=right+1
            print (area["NAME"], left, right)
            if left%2==1 and right%2==1 : return True
        print ("OUT")
        return False

    def getIntersections(self, x0, y0, x1, y1):
        # ray is (0.0)->(x,y) line
        # assume that we are inside
        intrs = []
        line=(x0+self.xu0, y0+self.yu0, x1+self.xu0, y1+self.yu0)

        for area in self.map["AREAS"] :
            for wall in area["WALLS"] :
                isect=self.intersectHor(line, wall["C"])
                if isect[0]==True :
                    intrs.append(isect[1])
        print (intrs)
        return False

    def intersectHor(self, y, line):
        res = (False, 0)
        x0, y0, x1, y1 = line
        if y0==y1 : #with hor line
            if y!=y0 : return res # no intersect
            else : return res # SPECIAL CASE :: OM LINE (TODO)
        if (y<y1 and y<=y0) or (y>y1 and y>=y0) : return res  # no intersect, note - left point not included (?)

        if x0>x1 : x0, y0, x1, y1 = x1, y1, x0, y0 #reorder

        x = x0+(y-y0)/(y1-y0)*(x1-x0)
        return (True, x)

    def intersectRay(self, line0, line1):
        # ray is (0.0)->(x,y) line
        return (False, (0, 0))

