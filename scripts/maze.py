import numpy as np

class Maze:
    class Node:
        def __init__(self, position=None):
            self.position = position
           
            self.neighbours =  [None, None, None, None]

        def __lt__(self, other):
            return (self.position < other.position) 

        def __gt__(self, other):
            return (self.position > other.position)

        def __le__(self, other):
            return (self < other) or (self == other)

        def __ge__(self, other):
            return (self > other) or (self == other)
        

    #first is 2
    #then 3
    def __init__(self,*args):
        if(len(args) == 2):
            maze = np.array(args[0])

            self.botEnd = args[1]
            self.width = maze.shape[0]
            self.height = maze.shape[1]

            self.start = None
            self.end = None
            nodecount = 0
            left = None
            toprownodes = [None] * self.width

            if(self.botEnd == False): #if flase then this maze for start to finish
                # Starting node - Finds where the starting position is at which will be the first opening in the first row
                for y in range(self.height):
                    if maze[0,y] == 0:
                        self.start = Maze.Node((0,y))
                        toprownodes[y] = self.start
                        nodecount += 1

                for x in range(1, self.width-1):

                    prev = False
                    current = False
                    next = maze[x,1] == 0

                    for y in range(1, self.height-1):
                        
                        prev = current
                        current = next
                        next = maze[x,y+1] == 0

                        n = None

                        if not current:
                            continue

                        if prev:
                            n = Maze.Node((x,y))
                            left.neighbours[1] = n
                            n.neighbours[3] = left
                            
                            if next:
                                left = n
                            else:
                                left = None

                        else:
                            n = Maze.Node((x,y))
                            left = n

                        if n != None:
                            if (maze[x-1,y] == 0):
                                t = toprownodes[y]
                                t.neighbours[2] = n
                                n.neighbours[0] = t

                            if (maze[x+1,y] == 0):
                                toprownodes[y] = n
                            
                            else:
                                toprownodes[y] = None

                            nodecount += 1


                # Ending node -- same as the first loop but here were trying to find the opening in the last
                for y in range(self.height):
                    if maze[-1,y] == 0:
                        self.end = Maze.Node((self.height-1,y))
                        t = toprownodes[y]
                        t.neighbours[2] = self.end
                        self.end.neighbours[0] = t
                        nodecount += 1
                        break

                self.nodecount = nodecount
                    

            else: # if the maze is for 2nd robot end to finish
                for y in range(self.height):
                    if maze[0,y] == 0:
                        self.end = Maze.Node((0,y))
                        toprownodes[y] = self.end
                        nodecount += 1

                for x in range(1, self.width-1):

                    prev = False
                    current = False
                    next = maze[x,1] == 0

                    for y in range(1, self.height-1):
                        
                        prev = current
                        current = next
                        next = maze[x,y+1] == 0

                        n = None

                        if not current:
                            continue

                        if prev:
                            n = Maze.Node((x,y))
                            left.neighbours[1] = n
                            n.neighbours[3] = left
                            
                            if next:
                                left = n
                            else:
                                left = None

                        else:
                            n = Maze.Node((x,y))
                            left = n

                        if n != None:
                            if (maze[x-1,y] == 0):
                                t = toprownodes[y]
                                t.neighbours[2] = n
                                n.neighbours[0] = t

                            if (maze[x+1,y] == 0):
                                toprownodes[y] = n
                            
                            else:
                                toprownodes[y] = None

                            nodecount += 1


                # start node -- same as the first loop but here were trying to find the opening in the last
                for y in range(self.height):
                    if maze[-1,y] == 0:
                        self.start = Maze.Node((self.height-1,y))
                        t = toprownodes[y]
                        t.neighbours[2] = self.start
                        self.start.neighbours[0] = t
                        nodecount += 1
                        break
            
            self.nodecount = nodecount

        elif(len(args) == 3):
            maze = np.array(args[0])
            self.width = maze.shape[0]
            self.height = maze.shape[1]
            self.start = None
            self.botEnd = args[2]
            loc = args[1]
            i = loc[0]
            j = loc[1]
            int(i)
            int(j)
            
            self.end = Maze.Node((i,j))
            nodecount = 0
            left = None
            toprownodes = [None] * self.width

            if(self.botEnd == False):
                for y in range(self.height):
                    if maze[0,y] == 0:
                        self.start = Maze.Node((0,y))
                        toprownodes[y] = self.start
                        nodecount += 1

                for x in range(1, self.width-1):

                    prev = False
                    current = False
                    next = maze[x,1] == 0

                    for y in range(1, self.height-1):
                        
                        prev = current
                        current = next
                        next = maze[x,y+1] == 0

                        n = None

                        if not current:
                            continue

                        if prev:
                            n = Maze.Node((x,y))
                            left.neighbours[1] = n
                            n.neighbours[3] = left
                            
                            if next:
                                left = n
                            else:
                                left = None

                        else:
                            n = Maze.Node((x,y))
                            left = n

                        if n != None:
                            if (maze[x-1,y] == 0):
                                t = toprownodes[y]
                                t.neighbours[2] = n
                                n.neighbours[0] = t

                            if (maze[x+1,y] == 0):
                                toprownodes[y] = n
                            
                            else:
                                toprownodes[y] = None

                            nodecount += 1
                self.nodecount = nodecount

            else:
                for y in range(self.height):
                    if maze[0,y] == 0:
                        #self.end = Maze.Node((0,y))
                        toprownodes[y] = self.end
                        nodecount += 1

                for x in range(1, self.width-1):
    
                    prev = False
                    current = False
                    next = maze[x,1] == 0

                    for y in range(1, self.height-1):
                        
                        prev = current
                        current = next
                        next = maze[x,y+1] == 0

                        n = None

                        if not current:
                            continue

                        if prev:
                            n = Maze.Node((x,y))
                            left.neighbours[1] = n
                            n.neighbours[3] = left
                            
                            if next:
                                left = n
                            else:
                                left = None

                        else:
                            n = Maze.Node((x,y))
                            left = n

                        if n != None:
                            if (maze[x-1,y] == 0):
                                t = toprownodes[y]
                                t.neighbours[2] = n
                                n.neighbours[0] = t

                            if (maze[x+1,y] == 0):
                                toprownodes[y] = n
                            
                            else:
                                toprownodes[y] = None

                            nodecount += 1

                for y in range(self.height):
                    if maze[-1,y] == 0:
                        self.start = Maze.Node((self.height-1,y))
                        t = toprownodes[y]
                        t.neighbours[2] = self.start
                        self.start.neighbours[0] = t
                        nodecount += 1
                        break
                     
                self.nodecount = nodecount