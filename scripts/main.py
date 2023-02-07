#!/usr/bin/env python3

from email.policy import strict
import rospy
from TurtlebotDriving import TurtlebotDriving
import threading
import numpy as np

import cv2
import matplotlib.pyplot as plt
import os
import yaml
import logging
import time


from maze import Maze
from algorithm.astar import AStar
from algorithm.breadthfirst import BFS
from algorithm.depthfirst import DFS
from algorithm.dijkstra import Dijkstra
from algorithm.wallfollower import WallFollower

config = {
    "map_dir": "map",
    "map_info":"map2.yaml",
    "algorithm":"bfs"
}

def moveBot1(pathS, bot1, i):
    for i in range(len(pathS)-1):
         bot1.move(pathS[i], pathS[i+1])
         
         
def moveBot2(pathE, bot2, i):
    for i in range(len(pathE)-1):
         bot2.move(pathE[i], pathE[i+1])

def makeDecise():
    print("\nWould you ike to enter a destination for the turtlebot to travel to?:\nInput y for yes and n for no\n")
    decise = input()
    print("You enetred in", decise)
    return decise



def createLocAvailable(arr):
    maze = np.array(arr)
    rows = len(maze)
    cols = len(maze[0])
    allowedLoc =[]
    for x in range(rows):
        for y in range(cols):
            if(maze[x,y] == 0):
                str = f"({x},{y})"
                allowedLoc.append(str)

    print("\n Input one of the locations you would like the robot to travel to: \n(type in x value then press enter and then type in y value and press enter)\n")
    print(allowedLoc)
    #here user inputs their selection
    while True:
        x = input()
        y = input()
        str = f"({x},{y})"
        check = checkIfValid(allowedLoc, str)

        #here we need to put in check
        if(check):
            x1 = int(x)
            y1 = int(y)
            print(f"You selected ({x},{y})")
            loc = [x1, y1]
            return loc;
            break
        else:
            print(f"\nThe coordinates ({x},{y}) are not valid please try again")

def checkIfValid(locations, str):
    length = len(locations)
    for i in range(length):
            if(locations[i] == str):
                return True
    return False






def main():
    # --------------------------------- Input ---------------------------------

    os.chdir(r'/home/jordanp36/catkin_ws/src/maze_solver')

    logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')

    # open map yaml file
    with open(os.path.join(config["map_dir"], config["map_info"])) as file:
        map_config = yaml.load(file, Loader=yaml.FullLoader)

    # Read image
    input = cv2.imread(os.path.join(config["map_dir"], map_config["image"]), -1)

    # Make wall = 1, path = 0
    input = (input != 254).astype(int)

    
    
    # Maze input
    plt.imshow(input, cmap='gray')
    plt.title("Maze Map")
    plt.axis("off")
    plt.show() 

    decise = makeDecise()

    if(decise == 'y'):
        print("For robot1:")
        selected1 = createLocAvailable(input)
        print("For robot2:")
        selected2 = createLocAvailable(input)
        print("Creating Maze...")
        t0 = time.time()
        mazeS = Maze(input, selected1, False)
        mazeE = Maze(input, selected2, True)
        t1 = time.time()

        print("\nNode Count:", mazeS.nodecount)
        print("Time elapsed:", t1-t0, "\n")
    else:
        print("Creating Maze...")
        t0 = time.time()
        mazeS = Maze(input, False)
        mazeE = Maze(input, True)
        t1 = time.time()

        print("\nNode Count:", mazeS.nodecount)
        print("Time elapsed:", t1-t0, "\n")

        



    
   

    
    # --------------------------------- Algorithm ---------------------------------

    if config["algorithm"].casefold() == "bfs":
        name = "Breadth First Search"
        algorithmS = BFS(mazeS)
        algorithmE = BFS(mazeE)

    
    elif config["algorithm"].casefold() == "dfs":
        name = "Depth First Search"
        algorithmS = DFS(mazeS)
        algorithmE = DFS(mazeE)


    elif config["algorithm"].casefold() == "dijkstra":
        name = "Dijkstra's Algorithm"
        algorithmS = Dijkstra(mazeS)
        algorithmE = Dijkstra(mazeE)


    elif config["algorithm"].casefold() == "astar":
        name = "A Star Algorithm"
        algorithmS = AStar(mazeS)
        algorithmE = AStar(mazeE)


    # Right wall or Left wall can be specified
    elif config["algorithm"].casefold() == "wallfollowing":
        name = "Wall Following"
        algorithm = WallFollower(speed=0.2, distance_wall=0.4, side="right")


    else:
        raise Exception('Algorithm specified not available (BFS, DFS, Astar, Dijkstra, WallFollowing)')
    
    # --------------------------------- Solve Maze ---------------------------------

    logging.info(f'''Starting Solve:
        Algorithm:         {name}
        Map:               {map_config["image"]}
        Map Resolution     {map_config["resolution"]}
        Map Resolution     {map_config["resolution"]}
    ''')

    # BFS, DFS, Djikstra, Astar
    if name != "Wall Following":

        t0 = time.time()
        pathS, countS, lengthS, completedS = algorithmS.solve()
        pathE, countE, lengthE, completedE = algorithmE.solve()
        t1 = time.time()

        if (completedS and completedE):
            print("Path found:")
            print(pathS)
            print("Node explored:", countS)
            print("Path length:", lengthS)
        
        else:
            print("\nNo path found")
            
        print("Time elapsed:", t1-t0, "\n")

        # --------------------------------- Output Image ---------------------------------

        input = (input==0).astype(int)
        for x,y in pathS:
            input[x,y] = 2

        # --------------------------------- Move Robot ---------------------------------

        i=0
        i2 = 0

        while i < (len(pathS)-2):
            if pathS[i][0] == pathS[i+1][0] == pathS[i+2][0] or pathS[i][1] == pathS[i+1][1] == pathS[i+2][1]:
                pathS.remove(pathS[i+1])
            else:
                i+=1

        while i2 < (len(pathE)-2):
            if pathE[i2][0] == pathE[i2+1][0] == pathE[i2+2][0] or pathE[i2][1] == pathE[i2+1][1] == pathE[i2+2][1]:
                pathE.remove(pathE[i2+1])
            else:
                i2+=1

            

        try:
            bot1 = TurtlebotDriving("tb3_0")
            bot2 = TurtlebotDriving("tb3_1")
            t0 = time.time()

            
            p_one = threading.Thread(target=moveBot1, args=(pathS,bot1,i,))
            p_two = threading.Thread(target=moveBot2, args=(pathE,bot2,i2,))
            p_one.start()
            p_two.start()
            p_one.join()
            p_two.join()


            t1 = time.time()

            print("Maze Solved!")
            bot1.plot_trajectory(name)
            bot2.plot_trajectory(name)
            bot1.relaunch()
            bot2.relaunch()
            print("Time taken :",t1-t0,"s\n")
            

        except rospy.ROSInterruptException:
            pass

        plt.imshow(input, cmap="gray")
        plt.title("Maze Solution")
        plt.axis("off")
        plt.show()


    # --------------------------------- Automous Solving ---------------------------------

    # Wall Following
    else:

        path, length, timetaken, completed = algorithm.run()

        if completed:
            print("Path found:")
            print(path)
            print("Path length:", length)
        
        else:
            print("\nNo path found")

        print("Time taken :",timetaken,"s\n")


        # --------------------------------- Output Image ---------------------------------
        
        input = (input==0).astype(int)

        for x,y in path:
            input[x,y] = 2

        plt.imshow(input)
        plt.title("Maze Solution")
        plt.axis("off")
        plt.show()


if __name__ == '__main__':
    main()
