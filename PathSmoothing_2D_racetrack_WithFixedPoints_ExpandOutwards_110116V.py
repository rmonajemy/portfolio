# #

# GOALS
#  - define a square path similar to a race track 
# -  smooth it out 
# - note that the initial and end point no longer need be fixed
# - use second newpath statement below to make smooth path extent outside of inner zone
#   - then even if corners are fixed a smooth path will be the outcome.  Otherwise, fixing corners will result in the original non-smooth path

from math import *  #  allow using multiply * directly
import numpy

import matplotlib
import matplotlib.pyplot as plt
import numpy

#%matplotlib inline  #  will plot in separate window with no argument




path = [[0, 0],
[0, 1],
[0, 2],
[0, 3],
[1, 3],
[2, 3],
[3, 3],
[4, 3],
[4, 2],
[4, 1],
[4, 0],
[3, 0],
[2, 0],
[1, 0],

]

FIXED = numpy.zeros(len(path))
FIXED[0] = 1
FIXED[3] = 1
FIXED[7] = 1
FIXED[10] = 1

# #

# weight_data factor:  affects how agressively NewPath points are being pulled towards Path
# weight_smooth factorL affects how aggressively newPath points are being pulled closed to each other
#       to smooth out the path


def smooth(path, weight_data = 0.3, weight_smooth = 0.5, tolerance = 0.1):

        newpath = [  [0 for col in range(len(path[0]))] for row in range(len(path))]
        for i in range(len(path)):
            for j in range(len(path[0])):              
                      newpath[i][j] = path[i][j]
        
        # LOOKS LIKE EFFECTIVELY EVERY DIMENTION IS BEING TREATED WITH SEPARATELY
        #  - we are doing smoothing in x dimention independetnly of y dimension
        change = tolerance
        N = 100 # total number of cycles to run max
        POINTS = len(path)
        DATA = numpy.zeros([N,1])
        
        #while (change >= tolerance):
        change = 0
            #for i in range(1,len(path)-1):  # exclude first and last points
        for i in range(N):  # dont exclude first and last points
                if i % POINTS == 0:
                    change = 0
                
                if FIXED[i % POINTS] == 0:  # if a point is fixed ( = 1) don't included it in optimization                
                    for j in range(len(path[0])):
                        ii = i % POINTS  #  create circular buffer point to current point
                        ii_plus_1 = (i+1) % POINTS
                        ii_plus_2 = (i+2) % POINTS
                        ii_minus_1 = (i-1) % POINTS 
                        ii_minus_2 = (i-2) % POINTS                         
                        
                        
                        d1 = weight_data * (path[ii][j] -  newpath[ii][j])
                        #d2 = weight_smooth * (newpath[ii_minus_1][j] + newpath[ii_plus_1][j]  - 2 * newpath[ii][j])
                        d2 = 0 # use second newpath statement below to make smooth path extent outside of inner zone
                        
                        
                        
                        DATA[i] = change
                        newpath[ii][j] += d1 + d2     
                        
                        # new code to make smooth path extend outside of block path
                        A = weight_smooth * ( newpath[ii_minus_1][j]  + newpath[ii_plus_1][j] - 2.0 * newpath[ii][j] ) 

                        B = 0.5 *weight_smooth * (2.0 * newpath[ii_minus_1][j] - newpath[ii_minus_2][j] - newpath[ii][j] )
                        
                        C = 0.5 *weight_smooth * (2.0 * newpath[ii_plus_1][j]  - newpath[ii_plus_2][j] - newpath[ii][j] )
                        
                        newpath[ii][j] += A + B + C
                        

                        change += abs(d1 + d2)

        #print('*****  Cycles needed to reach tolerance:', i)
        return (newpath, i, DATA) 

(newpath, i, DATA) = smooth(path)        # Make a deep copy of path into newpath
print('*****  Cycles needed to reach tolerance (OUTSIDE OF FUNCTION:', i)


#PLOT

X_path = numpy.zeros(len(path))
Y_path = numpy.zeros(len(path))

Xnew_path = numpy.zeros(len(path))
Ynew_path = numpy.zeros(len(path))


for i in range(len(path)):
    X_path[i] = path[i][0]
    Y_path[i] = path[i][1]

    Xnew_path[i] = newpath[i][0]
    Ynew_path[i] = newpath[i][1]



plt.figure(1)
plt.subplot(311)
plt.plot(X_path, Y_path,'bx',  Xnew_path, Ynew_path,'rx' )   
plt.axis([-1, 5, -3, 5])
plt.ylabel('Y')
plt.xlabel('X')
plt.title('path smoothing')
plt.grid(True)
plt.show()  

plt.subplot(312)
plt.plot(X_path, Y_path,'ko' )  
plt.axis([-1, 5, -1, 5]) 
plt.ylabel('Y')
plt.xlabel('X')
plt.grid(True)
plt.title('original path')
plt.show()  

plt.subplot(313)
plt.plot(DATA)  
#plt.axis([-1, 5, -1, 5]) 
plt.ylabel('error')
plt.xlabel('iteration')
plt.grid(True)
plt.title('Net Error')
plt.show()  


#for i in range(len(path)):
# print '['+ ', '.join('%.3f'%x for x in path[i]) +'] > ['+', '.join('%.3f'%x for x in newpath[i]) +']'