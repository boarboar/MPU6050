#from unitmap import UnitMap

class Planner:
    " Path planner"
    GRID_SZ=20 #cm
    GRID_DELTA=5 #cm
    def __init__(self, map):
        self.grid=None
        self.map=map

    def Plan(self):
        if self.grid is None:
            w=self.map.boundRect[2]-self.map.boundRect[0]
            h=self.map.boundRect[3]-self.map.boundRect[1]
            nx=w/self.GRID_SZ+1
            ny=h/self.GRID_SZ+1
            x0=self.map.boundRect[0]
            y0=self.map.boundRect[1]

            print('init grid NR=%s NC=%s...' % (ny, nx))

            self.grid = [[[x0+col*self.GRID_SZ,y0+row*self.GRID_SZ,0] for col in range(nx)] for row in range(ny)]
            for row in self.grid:
                for cell in row:
                    x, y =cell[0], cell[1]
                    area=[(x-self.GRID_DELTA,y-self.GRID_DELTA),
                          (x+self.GRID_SZ+self.GRID_DELTA, y-self.GRID_DELTA),
                          (x+self.GRID_SZ+self.GRID_DELTA, y+self.GRID_SZ+self.GRID_DELTA),
                          (x-self.GRID_DELTA, y+self.GRID_SZ+self.GRID_DELTA)]
                    cell[2]=self.map.At(area)


