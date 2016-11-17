
# part my Udacity "Artifical Intelligence for Robotics" class projects


#  GOALS
    #  peform A-star search on a discrete M x N grid
    #    0s mean that loation is passable
    #    1s mean that location cant be moved into   
    
    #  expand into neighboring states, keeping track of cost
    #  alwats start by expalnding states with lowest total cost 
    # dont expand eny block that has already been expanded into

# ----------
# User Instructions:
# 
# Define a function, search() that returns a list
# in the form of [optimal path length, row, col]. For
# the grid shown below, your function should output
# [11, 4, 5].
#
# If there is no valid path from the start point
# to the goal, your function should return the string
# 'fail'
# ----------

# Grid format:
#   0 = Navigable space
#   1 = Occupied space

import numpy

grid = [[0, 0, 0, 1, 0, 0],  # note that this would have been a 1x30 vector without the inner bracket on the 1x6 vectors
        [0, 1, 0, 1, 1, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 1, 0]]
        
heuristic = [[9, 8, 7, 6, 5, 4],
             [8, 7, 6, 5, 4, 3],
             [7, 6, 5, 4, 3, 2],
             [6, 5, 4, 3, 2, 1],
             [5, 4, 3, 2, 1, 0]]        
             
# un comment followign line to use plain seatch without A-star
#heuristic = numpy.zeros( [len(grid) ,  len(grid[0]) ] ) # rows x columns  

init = [0, 0]
goal = [len(grid)-1, len(grid[0])-1]
cost = 1

delta = [[-1, 0], # go up
         [ 0,-1], # go left  *UP AND LEFT ARE NEGATUIVE
         [ 1, 0], # go down  * DOWN AND RIGHT POSITION
         [ 0, 1]] # go right

delta_name = ['^', '<', 'v', '>']

def search(grid,init,goal,cost):
    closed = [[ 0 for row in range(len(grid[0]))  ] for col in range(len(grid)) ]   
    closed[init[0]][init[1]] = 1  # close starting point
    
    expand = [[ -1 for row in range(len(grid[0]))  ] for col in range(len(grid)) ]       
    action = [[ -1 for row in range(len(grid[0]))  ] for col in range(len(grid)) ]       
    
    x = init[0]
    y = init[1]
    g = 0
    h = heuristic[x][y]  
    f = g + h
    
    
    open = [[f,g,h,x,y]]

    found = False
    resign = False    
    count = 0
    
    while found is False and resign is False:
        # check if we still have elements on the open list
        if len(open) == 0:  # len returns number of rows in matrix,  number of length if vector
            resign = True
            print 'fail'
            
            
        else:
            # remove node with lowest total cost valye
            open.sort()         #sort array called open in ascending order
            open.reverse()      # just reverse the vectoir
            next = open.pop()   # take out last element from open and place in next
    
            x = next[3]
            y = next[4]
            g = next[1]
            
            expand[x][y] = count
            count += 1
            # check if we are done
            
            if x == goal[0] and y == goal[1]:
                found = True
                print next
                print '##### Search successful'
            else:
                    # expand winning element and add to new open list
                for i in range(len(delta)):
                    x2 = x + delta[i][0]
                    y2 = y + delta[i][1]
                    if x2 >= 0 and x2 < len(grid) and y2 >= 0 and y2 < len(grid[0]) :
                        if closed[x2][y2] == 0 and grid[x2][y2] == 0:
                            g2 = g + cost
                            
                            # print 'append list item'
                            h2 = heuristic[x2][y2]                            
                            f2 = g2 + h2
                            open.append([f2, g2, h2, x2, y2])                            
                            
                            #print [g2, x2, y2]
                            closed[x2][y2] = 1
                            action[x2][y2] = i
                            
    for i in range(len(expand)):
        print expand[i]           
        
    policy = [ [' ' for row in range(len(grid[0]))] for col in range(len(grid))  ]
    x = goal[0]
    y = goal[1]
    policy[x][y] = '*'
    while (x != init[0]) or (y != init[1]):
        x2 = x - delta[action[x][y]][0]    
        y2 = y - delta[action[x][y]][1]
        policy[x2][y2] = delta_name[action[x][y]]
        x = x2
        y = y2
        
    for i in range(len(policy)):
        print policy[i]        
        
        
    return
    #return path
    
    
search(grid,init,goal,cost)    
    
    
