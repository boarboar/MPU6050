import timeit
#from unitmap import UnitMap
# grid (x, y, state)

class Planner:
    " Path planner"

    def __init__(self, map, LogString, LogErrorString):
        self.map=map
        self.LogString=LogString
        self.LogErrorString=LogErrorString
        self.grid=None
        self.start_pos=None
        self.target_pos=None
        self.start_cell=None
        self.target_cell=None
        self.path=[]
        self.spath=[]

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
        self.start_cell=(int((self.start_pos[1]-cell0[1])/self.grid_sz), int((self.start_pos[0]-cell0[0])/self.grid_sz))
        self.target_cell=(int((self.target_pos[1]-cell0[1])/self.grid_sz), int((self.target_pos[0]-cell0[0])/self.grid_sz))
        return True


    def Plan(self, verbose=True):
        if self.grid is None:

            x0=self.map.boundRect[0]
            y0=self.map.boundRect[1]

            #self.LogString('init grid NR=%s NC=%s...' % (ny, nx))
            #   Cell struct:
            #   0: x
            #   1: y
            #   2: status
            #   3: closed
            #   4: expand
            #   5: action
            #   6: heuristics
            #   7: weight

            start_time = timeit.default_timer()
            """
            self.grid = [[[x0+col*self.GRID_SZ,y0+row*self.GRID_SZ,None,None,None,None,None,0] for col in range(nx)] for row in range(ny)]
            for row in self.grid:
                for cell in row:
                    x, y =cell[0], cell[1]
                    area=[(x-self.GRID_DELTA,y-self.GRID_DELTA),
                          (x+self.GRID_SZ+self.GRID_DELTA, y-self.GRID_DELTA),
                          (x+self.GRID_SZ+self.GRID_DELTA, y+self.GRID_SZ+self.GRID_DELTA),
                          (x-self.GRID_DELTA, y+self.GRID_SZ+self.GRID_DELTA)]
                    cell[2]=self.map.At(area)
            """
            self.grid_sz = self.map.GRID_SZ
            self.grid = []
            for mrow in self.map.grid:
                row = []
                self.grid.append(row)
                for mcell in mrow:
                    row.append([mcell[0], mcell[1], mcell[2], None,None,None,None,None,0])

            print('init grid NR=%s NC=%s...' % (len(self.grid), len(self.grid[0])))

            for row in self.grid:
                for cell in row:
                    if cell[2]==1 : cell[7]=1
                    else : cell[7]=0


            delta = [[-1, 0 ], # go down
                [ 0, -1], # go left
                [ 1, 0 ], # go up
                [ 0, 1 ]] # go right

            # ad weights to wall adjasent cells

            for rep in range(3) :
                #w2 = [[0 for col in range(nx)] for row in range(ny)]
                w2 = [[0 for col in range(len(self.grid[0]))] for row in range(len(self.grid))]
                for irow in range(len(self.grid)):
                    for icol in range(len(self.grid[irow])):
                        for i in range(len(delta)):
                            irow2 = irow + delta[i][0]
                            icol2 = icol + delta[i][1]
                            if irow2 >= 0 and irow2 < len(self.grid) and icol2 >=0 and icol2 < len(self.grid[irow]) and self.grid[irow2][icol2][7]>0:
                                w2[irow][icol]=self.grid[irow][icol][7]+self.grid[irow2][icol2][7]
                for irow in range(len(self.grid)):
                    for icol in range(len(self.grid[irow])): self.grid[irow][icol][7]=w2[irow][icol]


            print('grid inited in %s s' % (round(timeit.default_timer() - start_time, 2)))

        self.SetPath()

        if self.start_cell is None or self.target_cell is None :
            if verbose : self.LogErrorString("Start or Target not defined")
            return False

        if self.grid[self.start_cell[0]][self.start_cell[1]][2] != 0 :
            if verbose : self.LogErrorString("Start is occupied")
            return False

        if self.grid[self.target_cell[0]][self.target_cell[1]][2] != 0 :
            if verbose : self.LogErrorString("Target is occupied")
            return False

        if verbose : self.LogString("Do planning from (%s, %s) to (%s, %s)" %
                       (self.start_cell[0], self.start_cell[1], self.target_cell[0], self.target_cell[1]))

        del self.path[:]
        del self.spath[:]

        start_time = timeit.default_timer()
        if not self.AStarPlanning(verbose) : return False
        plan_time = timeit.default_timer() - start_time
        start_time = timeit.default_timer()
        self.SmoothPath(verbose)
        smooth_time = timeit.default_timer() - start_time
        if verbose: self.LogString('Smoothed in %s s' % (round(timeit.default_timer() - start_time, 2)))

        print('Planning done: in %s s (plan %s, smooth %s)' % (round(plan_time+smooth_time, 2),round(plan_time, 2),round(smooth_time, 2)))

        return True

    def RePlanOnMove(self, pos, verbose=True):
        if self.grid is None:
            return False
        if self.start_cell is None or self.target_cell is None :
            return False
        cell0=self.grid[0][0]
        new_start_cell=(int((pos[1]-cell0[1])/self.grid_sz), int((pos[0]-cell0[0])/self.grid_sz))
        if self.start_cell is not None and self.start_cell[0]==new_start_cell[0] and self.start_cell[1]==new_start_cell[1] :
            return True
        self.SetStart(pos)
        return self.Plan(verbose)


    def SimplePlanning(self):

        for row in self.grid:
            for cell in row:
                cell[3]=0 #closed
                cell[4]=-1 #expand
                cell[5]=-1 #action
                cell[6]=0 #heuristics


        cost = 1
        delta = [[-1, 0 ], # go down
         [ 0, -1], # go left
         [ 1, 0 ], # go up
         [ 0, 1 ], # go right
                 [1,1],[-1,-1],[1,-1],[-1,1] #go diagonal
                 ]
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
                self.LogErrorString('fail')
                return False
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
            self.LogErrorString('fail')
            return False

        self.LogString('Done expansion')

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

        #print(self.path)
        self.LogString('Done path')

        return True

    def AStarPlanning(self, verbose=True):

        start_time = timeit.default_timer()

        trow = self.target_cell[0]
        tcol = self.target_cell[1]

        for irow in range(len(self.grid)):
            for icol in range(len(self.grid[irow])):
                cell=self.grid[irow][icol]
                cell[3]=0 #closed
                cell[4]=-1 #expand
                cell[5]=-1 #action
                cell[6]=abs(trow-irow)+abs(tcol-icol) #heuristics

        #cost = 1
        tcost=2
        """
        delta = [[-1, 0 ], # go down
         [ 0, -1], # go left
         [ 1, 0 ], # go up
         [ 0, 1 ]] # go right
         """
        delta = [
            [-1, 0, 1 ], # go down
            [ 0, -1, 1], # go left
            [ 1, 0, 1 ], # go up
            [ 0, 1, 1 ] # go right
         ,[1,1, 1.4],[-1,-1, 1.4],[1,-1, 1.4],[-1,1, 1.4] #go diagonal, higher weights
                 ]
        irow = self.start_cell[0]
        icol = self.start_cell[1]
        #closed[init[0]][init[1]] = 1
        self.grid[irow][icol][3]=1
        g = 0
        #h=heuristic[x][y]
        h=self.grid[irow][icol][6]
        f = g+h

        open = [[f, g, h, irow, icol]]

        found = False  # flag that is set when search is complete
        resign = False # flag set if we can't find expand
        count = 0

        while not found and not resign:
            if len(open) == 0:
                resign = True
                self.LogErrorString('fail')
                return False
            else:
                open.sort()
                open.reverse()
                next = open.pop()


                g = next[1]
                irow = next[3]
                icol = next[4]
                self.grid[irow][icol][4]=count
                # expand[x][y] = count
                count+=1

                if irow == self.target_cell[0] and icol == self.target_cell[1]:
                    found = True
                else:
                    for i in range(len(delta)):
                        irow2 = irow + delta[i][0]
                        icol2 = icol + delta[i][1]
                        if irow2 >= 0 and irow2 < len(self.grid) and icol2 >=0 and icol2 < len(self.grid[0]):
                            #if closed[x2][y2] == 0 and grid[x2][y2] == 0:
                            if self.grid[irow2][icol2][3] == 0 and self.grid[irow2][icol2][2] == 0:
                                turncost=0
                                if self.grid[irow][icol][5]!=i : turncost=tcost # avoid turns
                                cost=delta[i][2]
                                g2 = g + cost + self.grid[irow2][icol2][7] + turncost # + weight
                                #h2=heuristic[x2][y2]
                                h2=self.grid[irow2][icol2][6]
                                f2 = g2+h2
                                open.append([f2, g2, h2, irow2, icol2])
                                #action[x2][y2]=i
                                #closed[x2][y2] = 1
                                self.grid[irow2][icol2][3] = 1
                                self.grid[irow2][icol2][5] = i #action

        if resign:
            if verbose : self.LogErrorString('fail')
            return False

        if verbose : self.LogString('Done expansion')

        del self.path[:]

        irow = self.target_cell[0]
        icol = self.target_cell[1]
        self.path.append([irow, icol])

        while not (irow==self.start_cell[0] and icol==self.start_cell[1]) :
            i = self.grid[irow][icol][5] #action
            irow2 = irow - delta[i][0]
            icol2 = icol - delta[i][1]
            self.path.append([irow2, icol2])
            irow=irow2
            icol=icol2

        #print(self.path)
        if verbose : self.LogString('Done path')
        if verbose : self.LogString('Planning done in %s s' % (round(timeit.default_timer() - start_time, 2)))

        #print('Planning done in %s s' % (round(timeit.default_timer() - start_time, 2)))

        return True

    def SmoothPath(self, verbose=True):
        path=[]
        #weight_data = 0.3
        #weight_smooth = 0.7
        weight_data = 0.1
        weight_smooth = 0.9
        #tolerance = 0.000001
        tolerance = 0.0001
        for step in self.path :
            cell=self.grid[step[0]][step[1]]
            x, y =cell[0], cell[1]
            #self.spath.append((x+self.GRID_SZ/2, y+self.GRID_SZ/2))
            path.append([x+self.grid_sz/2, y+self.grid_sz/2])
            self.spath.append([x+self.grid_sz/2, y+self.grid_sz/2])

        tol = 999
        while tol>=tolerance:
            tol=0.0
            # TODO - avoid occupied cells (get newpos (2 crds), check occupancy, replace if not occupied)
            for i in range(1, len(self.spath)-1):
                for k in range(2):
                    n=self.spath[i][k]+\
                      weight_data*(path[i][k]-self.spath[i][k])+\
                      weight_smooth*(self.spath[i+1][k]+self.spath[i-1][k]-2.0*self.spath[i][k])
                    if n-self.spath[i][k]>tol:
                        tol=n-self.spath[i][k]
                    if self.spath[i][k]-n>tol:
                        tol=self.spath[i][k]-n
                    self.spath[i][k] = n
        self.spath.reverse()