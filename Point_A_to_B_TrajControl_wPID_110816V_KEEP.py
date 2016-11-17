
# part my Udacity "Artifical Intelligence for Robotics" class projects

# -----------
# User Instructions
#
# Implement a P controller by running 100 iterations
# of robot motion. The desired trajectory for the 
# robot is the x-axis. The steering angle should be set
# by the parameter tau so that:
#
# steering = -tau * crosstrack_error
#
# Note that tau is called "param" in the function
# below.
#
# Your code should print output that looks like
# the output shown in the video. That is, at each step:
# print myrobot, steering
#
# Only modify code at the bottom!
# ------------
 
from math import *
import random
import numpy

import matplotlib
import matplotlib.pyplot as plt
# ------------------------------------------------
# 
# this is the robot class
#

class robot:

    # --------
    # init: 
    #    creates robot and initializes location/orientation to 0, 0, 0
    #

    def __init__(self, length = 8.0):
        self.x = 0.0
        self.y = 0.0
        self.orientation = 0.0
        self.length = length
        self.steering_noise = 0.0
        self.distance_noise = 0.0
        self.steering_drift = 0.0

    # --------
    # set: 
    #	sets a robot coordinate
    #

    def set(self, new_x, new_y, new_orientation):

        self.x = float(new_x)
        self.y = float(new_y)
        self.orientation = float(new_orientation) % (2.0 * pi)


    # --------
    # set_noise: 
    #	sets the noise parameters
    #

    def set_noise(self, new_s_noise, new_d_noise):
        # makes it possible to change the noise parameters
        # this is often useful in particle filters
        self.steering_noise = float(new_s_noise)
        self.distance_noise = float(new_d_noise)

    # --------
    # set_steering_drift: 
    #	sets the systematical steering drift parameter
    #

    def set_steering_drift(self, drift):
        self.steering_drift = drift
        
    # --------
    # move: 
    #    steering = front wheel steering angle, limited by max_steering_angle
    #    distance = total distance driven, most be non-negative

    def move(self, steering, distance, 
             tolerance = 0.001, max_steering_angle = pi / 4.0):

        if steering > max_steering_angle:
            steering = max_steering_angle
        if steering < -max_steering_angle:
            steering = -max_steering_angle
        if distance < 0.0:
            distance = 0.0


        # make a new copy
        res = robot()
        res.length         = self.length
        res.steering_noise = self.steering_noise
        res.distance_noise = self.distance_noise
        res.steering_drift = self.steering_drift

        # apply noise
        steering2 = random.gauss(steering, self.steering_noise)
        distance2 = random.gauss(distance, self.distance_noise)

        # apply steering drift
        steering2 += self.steering_drift

        # Execute motion
        turn = tan(steering2) * distance2 / res.length

        if abs(turn) < tolerance:

            # approximate by straight line motion

            res.x = self.x + (distance2 * cos(self.orientation))
            res.y = self.y + (distance2 * sin(self.orientation))
            res.orientation = (self.orientation + turn) % (2.0 * pi)

        else:

            # approximate bicycle model for motion

            radius = distance2 / turn
            cx = self.x - (sin(self.orientation) * radius)
            cy = self.y + (cos(self.orientation) * radius)
            res.orientation = (self.orientation + turn) % (2.0 * pi)
            res.x = cx + (sin(res.orientation) * radius)
            res.y = cy - (cos(res.orientation) * radius)

        return res


    def  cte(self, radius):  # define cross track error for a race track
                            # length =2R & with = 2R (excluding the circles),  half circle on each cide
                            
        if self.x < radius:
            cte = sqrt((self.x - radius)**2 + (self.y - radius)**2)
        elif self.x > 3.0 * radius:
            cte = sqrt((self.x - 3.0*radius)**2 + (self.y - radius)**2)
        elif self.y > radius:
            cte = self.y - 2.0*radius
        else:
            cte = - self.y
        return cte

    def __repr__(self):
        return '[x=%.5f y=%.5f orient=%.5f]'  % (self.x, self.y, self.orientation)

############## ADD / MODIFY CODE BELOW ####################
    
# ------------------------------------------------------------------------
#
# run - does a single control run




# calcualte next target during current sample
def Calculate_Next_Target():
    
    # calculae Xct,Yct = junction of perpendicular line from (x,y) to INIT to TARGET path


    """
    X0,  Y0

    Xt,  Yt
    
    
    Y = y - Y0
    X = x - X0
    
    alpha = atan2( (Yt - Y0) / (Xt - X0))
    TAN_alpha =  (Yt - Y0) / (Xt - X0)
    X1 = Y / TAN_alpha
    X2 = X - x1
    H = X2 * sin(alpha)  # distance from (x,y) to INIT to TARGET path
    
    dX = H * sin(alpha)
    dY = H * cos(alpha)

    Xct = x - dX
    Yct = y + dY
    
    """
    
        
    
    
    
    
    
    X_next = TARGET_VECTOR[0]  # for now assume next target is the final target
    Y_next = TARGET_VECTOR[1]
    TARGET = [X_next, Y_next]
    return TARGET


# calculate cross track error based on deisred policy 
def Calculate_CTE(TAR, myrobot1):
    crosstrack_error = myrobot1.x - TAR[0]  #
    crosstrack_error += myrobot1.y - TAR[1]
    
    
    
    #crosstrack_error = myrobot.y - TAR[1]
    return crosstrack_error


# subroutine to run a robot from a starting point with target of getting to A certain lcation with PID control

def run(param_p, param_d, param_i, myrobot, N, TARGET_V):  # trajectory:  race track with R=radius and lenght of flat oath = 2R
    
  
    X0 = myrobot.x
    Y0 = myrobot.y
    O0 = myrobot.orientation
    steering = 0    
   
    #myrobot.set(0.0, radius, 0.0)
    speed = 1.0 # M/S, motion distance is equal to speed (we assume time = 1)
    y_last = Y0
    x_last = X0
    
 
    delta_y_integral = 0

    #myrobot.set_steering_drift(10.0 * pi/180)  # fixed drift on steering


    #SAVE_y = numpy.zeros([1,100])
    #SAVE_steering = numpy.zeros([1,100])
    SAVE =  numpy.zeros([N,4])
    
    
    TARGET_x = TARGET_V[0]  
    TARGET_y = TARGET_V[1]
     
    crosstrack_error = 0.0  #initialize
    
    int_cte = 0.0 
    
    for i in range(N): #assume 1s updates

        SAVE[i][0] = myrobot.x
        
        SAVE[i][1] = myrobot.y
       
        SAVE[i][2] = steering
        
        orientation_s = myrobot.orientation
        if  orientation_s  > pi:
            orientation_s = orientation_s - 2*pi
        SAVE[i][3] = orientation_s
    
        #print(myrobot.steering_drift)    
        #1  calculate next target
        XY_target = Calculate_Next_Target()  #  calculate next targets Xt, Yt for thsi sample (not final target)
                                             # return a 1x2 vector
        
        
        diff_cte = -crosstrack_error

        #2  calculate cross track error based on design policy
        crosstrack_error = Calculate_CTE(XY_target, myrobot)  
               
        #crosstrack_error = myrobot.x - TARGET_x
        #crosstrack_error = myrobot.y - TARGET_y

        diff_cte += crosstrack_error
        int_cte += crosstrack_error
    
       
        steering = - (param_p * crosstrack_error + param_d*(diff_cte) + param_i*int_cte )  # proportional control
        if steering > pi/4:
            steering = pi/4
        if steering < -pi/4:
            steering = -pi/4
            
        x_last = myrobot.x    
        y_last = myrobot.y
        #print('Y =', y_last)
        
        # if we are within 15 meters of target then stop
        distance_to_target = sqrt( (myrobot.x - TARGET_x)**2  + (myrobot.y - TARGET_y)**2  )
        if distance_to_target < 15:
                    speed = 0
         
        distance = speed * 1 # speed * sampling time = distance travelled during samping interval   
    
    
        myrobot = myrobot.move(steering, distance) # =>  x and y and orient will get updated
        #myrobot.set(newR.x, newR.y, newR.orientation)
        
        

       
        #print myrobot.y
        
    return SAVE


myrobot = robot(10.0);  # set lenght at 10 meters


#  SET INIT LOCATION OF ROBOT
X0 = 0.0
Y0 = 0.0
O0 = 2
INIT_VECTOR = [X0, Y0, O0]
myrobot.set(X0, Y0, O0)

#  SET INIT LOCATION OF ROBOT
Xfinal = 60.0
Yfinal = -40.0
Ofinal = 0
TARGET_VECTOR = [Xfinal, Yfinal, Ofinal]



N = 250  # number of samples,  1s each

DATA = run(0.2, 5.0, 0.0, myrobot, N, TARGET_VECTOR) # KEEP P, D, I gains # call function with parameter tau of 0.1 and print results
#  P gain = 0.2,  D gain = 10 and I gain = 0 seem to work just fine


#DATA = run(1.0, 1.0, 0.0, radius) 

X = numpy.zeros(len(DATA))
Y = numpy.zeros(len(DATA))
O = numpy.zeros(len(DATA))
S = numpy.zeros(len(DATA))

NN = len(DATA)
for i in range(NN):
    X[i] = DATA[i][0]  #  x axis
    Y[i] = DATA[i][1]  #  y
    S[i] = DATA[i][2]  # steering
    O[i] = DATA[i][3]  #  orientation
   

plt.figure(1)
plt.subplot(111)
plt.plot(X, Y)   
plt.axis([-20, 70, -70, 20])
plt.ylabel('y')
plt.xlabel('x')
plt.title('XY position')
plt.grid(True)
plt.show()  


plt.figure(2)

plt.subplot(411)
plt.plot(X)  
#plt.axis([0, NN, -50, 50]) 
plt.ylabel('x')
#plt.xlabel('Time, seconds')
plt.title('X')
plt.grid(True)
plt.show()  


plt.subplot(412)
plt.plot(Y)  
#plt.axis([0, NN, -50, 50]) 
plt.ylabel('y')
#plt.xlabel('Time, seconds')
plt.title('Y')
plt.grid(True)
plt.show()  






plt.subplot(413)
plt.plot(S * 180/pi )  
plt.axis([0, NN, -50, 50]) 
plt.ylabel('deg')
#plt.xlabel('Time, seconds')
plt.title('Steering Angle')
plt.grid(True)
plt.show()  

plt.subplot(413)
plt.plot(O * 180/pi )  
#plt.axis([0, NN, -45, 45]) 
plt.ylabel('deg')
plt.xlabel('Time, seconds')
plt.title('Orientation')
plt.grid(True)
plt.show()  

#plt.figure(3)
#plt.plot(S*180/pi)
