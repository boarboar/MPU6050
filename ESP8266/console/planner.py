#from unitmap import UnitMap
# grid (x, y, state)

class Planner:
    " Path planner"
    GRID_SZ=20 #cm
    GRID_DELTA=5 #cm
    def __init__(self, map):
        self.grid=None
        self.map=map
        self.start_pos=None
        self.target_pos=None
        self.start_cell=None
        self.target_cell=None
        self.path=[]

    def SetStart(self, pos):
        self.start_pos=pos
        return self.SetPath()

    def SetTarget(self, pos):
        self.target_pos=pos
        return self.SetPath()

    def SetPath(self):
        if self.grid is None or self.start_pos is None or self.target_pos is None:
            self.start_cell=None
            self.target_cell=None
            return False
        cell0=self.grid[0][0]
        self.start_cell=(int((self.start_pos[1]-cell0[1])/self.GRID_SZ), int((self.start_pos[0]-cell0[0])/self.GRID_SZ))
        self.target_cell=(int((self.target_pos[1]-cell0[1])/self.GRID_SZ), int((self.target_pos[0]-cell0[0])/self.GRID_SZ))
        print('from (%s, %s) to (%s, %s)' % (self.start_cell[0], self.start_cell[1], self.target_cell[0], self.target_cell[1]))
        return True


    def Plan(self):
        if self.grid is None:
            w=self.map.boundRect[2]-self.map.boundRect[0]
            h=self.map.boundRect[3]-self.map.boundRect[1]
            nx=w/self.GRID_SZ+1
            ny=h/self.GRID_SZ+1
            x0=self.map.boundRect[0]
            y0=self.map.boundRect[1]

            print('init grid NR=%s NC=%s...' % (ny, nx))

            self.grid = [[[x0+col*self.GRID_SZ,y0+row*self.GRID_SZ,None,None,None,None] for col in range(nx)] for row in range(ny)]
            for row in self.grid:
                for cell in row:
                    x, y =cell[0], cell[1]
                    area=[(x-self.GRID_DELTA,y-self.GRID_DELTA),
                          (x+self.GRID_SZ+self.GRID_DELTA, y-self.GRID_DELTA),
                          (x+self.GRID_SZ+self.GRID_DELTA, y+self.GRID_SZ+self.GRID_DELTA),
                          (x-self.GRID_DELTA, y+self.GRID_SZ+self.GRID_DELTA)]
                    cell[2]=self.map.At(area)

        self.SetPath()


        if self.start_cell is None or self.start_cell is None :
            print("Start or Target not defined")
            return False

        if self.grid[self.start_cell[0]][self.start_cell[1]][2] != 0 :
            print("Start is occupied")
            return False

        if self.grid[self.target_cell[0]][self.target_cell[1]][2] != 0 :
            print("Target is occupied")
            return False

        print("Do planning")

        for row in self.grid:
            for cell in row:
                cell[3]=0 #closed
                cell[4]=-1 #expand
                cell[5]=-1 #action


        cost = 1
        delta = [[-1, 0 ], # go down
         [ 0, -1], # go left
         [ 1, 0 ], # go up
         [ 0, 1 ]] # go right
        irow = self.start_cell[0]
        icol = self.start_cell[1]
        #closed[init[0]][init[1]] = 1
        self.grid[irow][icol][3]=1
        g = 0
        open = [[g, irow, icol]]

        found = False  # flag that is set when search is complete
        resign = False # flag set if we can't find expand

        while not found and not resign:
            if len(open) == 0:
                resign = True
                print('fail')
                return False
                #return 'fail'
            else:
                open.sort()
                open.reverse()
                next = open.pop()
                irow = next[1]
                icol = next[2]
                g = next[0]

                #expand[x][y]=g
                self.grid[irow][icol][4]=g

                if irow == self.target_cell[0] and icol == self.target_cell[1]:
                    found = True
                else:
                    for i in range(len(delta)):
                        irow2 = irow + delta[i][0]
                        icol2 = icol + delta[i][1]
                        if irow2 >= 0 and irow2 < len(self.grid) and icol2 >=0 and icol2 < len(self.grid[0]):
                            #if closed[x2][y2] == 0 and grid[x2][y2] == 0:
                            if self.grid[irow2][icol2][3] == 0 and self.grid[irow2][icol2][2] == 0:
                                g2 = g + cost
                                open.append([g2, irow2, icol2])
                                #action[x2][y2]=i
                                #closed[x2][y2] = 1
                                self.grid[irow2][icol2][3] = 1
                                self.grid[irow2][icol2][5] = i #action

        if resign:
            print('fail')
            return False
            #return 'fail'

        print('Done expansion')

        del self.path[:]

        irow = self.target_cell[0]
        icol = self.target_cell[1]
        self.path.append((irow, icol, '*'))

        while not (irow==self.start_cell[0] and icol==self.start_cell[1]) :
            i = self.grid[irow][icol][5] #action
            irow2 = irow - delta[i][0]
            icol2 = icol - delta[i][1]
            self.path.append((irow2, icol2, '*'))
            irow=irow2
            icol=icol2

        print(self.path)
        print('Done path')

        return True

