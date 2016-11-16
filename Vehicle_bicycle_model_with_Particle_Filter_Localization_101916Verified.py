

# -----------------
# USER INSTRUCTIONS
#
# Write a function in the class robot called move()
#
# that takes self and a motion vector (this
# motion vector contains a steering* angle and a
# distance) as input and returns an instance of the class
# robot with the appropriate x, y, and orientation
# for the given motion.
#
# *steering is defined in the video
# which accompanies this problem.
#
# For now, please do NOT add noise to your move function.
#
# Please do not modify anything except where indicated
# below.
#
# There are test cases which you are free to use at the
# bottom. If you uncomment them for testing, make sure you
# re-comment them before you submit.

from math import *
import random
# --------
# 
# the "world" has 4 landmarks.
# the robot's initial coordinates are somewhere in the square
# represented by the landmarks.
#
# NOTE: Landmark coordinates are given in (y, x) form and NOT
# in the traditional (x, y) format!

landmarks  = [[0.0, 100.0], [0.0, 0.0], [100.0, 0.0], [100.0, 100.0]] # position of 4 landmarks
world_size = 100.0 # world is NOT cyclic. Robot is allowed to travel "out of bounds"


max_steering_angle = pi/4 # You don't need to use this value, but it is good to keep in mind the limitations of a real car.
bearing_noise = 0.1 # Noise parameter: should be included in sense function.
steering_noise = 0.1 # Noise parameter: should be included in move function.
distance_noise = 5.0 # Noise parameter: should be included in move function.

tolerance_xy = 15.0 # Tolerance for localization in the x and y directions.
tolerance_orientation = 0.25 # Tolerance for orientation.

# ------------------------------------------------
# 
# this is the robot class
#

class robot:

    # --------

    # init: 
    #	creates robot and initializes location/orientation 
    #

    def __init__(self, length = 10.0):  #initialize lenght to 10
    
        self.x = random.random() * world_size # initial x position
        self.y = random.random() * world_size # initial y position
        self.orientation = random.random() * 2.0 * pi # initial orientation
        self.length = length # length of robot
        self.bearing_noise  = 0.0 # initialize bearing noise to zero
        self.steering_noise = 0.0 # initialize steering noise to zero
        self.distance_noise = 0.0 # initialize distance noise to zero
    
    def __repr__(self):
        return '[x=%.4s y=%.4s orient=%.4s]' % (str(self.x), str(self.y), str(self.orientation))  # orientation
        
    # --------
    # set: 
    #	sets a robot coordinate
    #

    def set(self, new_x, new_y, new_orientation):

        if new_orientation < 0 or new_orientation >= 2 * pi:
            raise ValueError, 'Orientation must be in [0..2pi]'
        self.x = float(new_x)
        self.y = float(new_y)
        self.orientation = float(new_orientation)


    # --------
    # set_noise: 
    #	sets the noise parameters
    #

    def set_noise(self, new_b_noise, new_s_noise, new_d_noise):
        # makes it possible to change the noise parameters
        # this is often useful in particle filters
        self.bearing_noise  = float(new_b_noise)
        self.steering_noise = float(new_s_noise)
        self.distance_noise = float(new_d_noise)
    
    ############# ONLY ADD/MODIFY CODE BELOW HERE ###################

    # --------
    # move:
    #   move along a section of a circular path according to motion
    #
    
    def move(self, motion, tolerance = 0.001): # Do not change the name of this function
        # motion[0] = alpha
        # motion[1] = distance moved 


        # these two are the only known variables / rest must be calculated 
        steering = motion[0]    #  = alpha = STEERING ANGLE
        distance = motion[1]    #  = angualr distnace moved by robot's rear wheel


        #???????????????????????
        if abs(steering) > max_steering_angle:    # WHATS THIS?
        
                 raise ValueError,  'Exceeding max steering angle'
         
        if distance < 0.0:
                 raise ValueError, 'moving backwards is not valid'
        
        
        res = robot()
        res.length           = self.length
        res.bearing_noise    = self.bearing_noise
        res.steering_noise   = self.steering_noise
        res.distance_noise   = self.distance_noise
            
        #apply noise
        steering2 = random.gauss(steering, self.steering_noise)  # just add noise with variance steering_noise
        distance2 = random.gauss(distance, self.steering_noise)  #   * just one random value is added to each
        

        # Execute motion  (for given steering angle and distance of movement)
        turn = tan(steering2) * distance2 / res.length #  = beta, rotation angle of back tire after move
        # ???????????? NEED TO PROVE THIS TOO
        #print('turn angle = ', turn)

        if abs(turn) < tolerance:
            # approximate by straight line motion
            res.x = self.x  + (distance2 * cos(self.orientation)) 
            res.y = self.y  + (distance2 * sin(self.orientation))
            res.orientation  = (self.orientation + turn) % (2.0 * pi) 
            
        else:
            radius = distance2 / turn  # radius of imaginary center of rotation

            #???? NOT CLEAR WHY SIN IS USED IN FIRST STATEMENT WHILE COS MAKES MORE SENSE
            cx = self.x - (sin(self.orientation) * radius)
            cy = self.y + (cos(self.orientation) * radius)
            res.orientation = (self.orientation + turn)  % (2.0 * pi)  # calculate new orientation
            res.x = cx + (sin(res.orientation) * radius)
            res.y = cy - (cos(res.orientation) * radius)
            
            

        return res # make sure your move function returns an instance
                      # of the robot class with the correct coordinates.
                 
                 
    def sense(self, add_noise = 1):
        Z = []
        for i in range(len(landmarks)):
            bearing = atan2( (landmarks[i][0] - self.y), (landmarks[i][1] - self.x)) - self.orientation
            if add_noise:
                    bearing += random.gauss(0.0, self.bearing_noise)
            bearing %= 2.0 * pi
            Z.append(bearing)
        return Z
                 
              
# --------
    # measurement_prob
    #    computes the probability of a measurement
    #  

    def measurement_prob(self, measurements):

        # calculate the correct measurement
        predicted_measurements = self.sense(0) # Our sense function took 0 as an argument to switch off noise.


        # compute errors
        error = 1.0
        for i in range(len(measurements)):
            error_bearing = abs(measurements[i] - predicted_measurements[i])
            error_bearing = (error_bearing + pi) % (2.0 * pi) - pi # truncate
            

            # update Gaussian
            error *= (exp(- (error_bearing ** 2) / (self.bearing_noise ** 2) / 2.0) /  
                      sqrt(2.0 * pi * (self.bearing_noise ** 2)))

        return error
    
# --------
#
# extract position from a particle set
# 

def get_position(p):  # input is a  particle vector consisting of many particles
                        # get average of x, y, orient of all particles
                        # if particles are close to target these averages will be also reflect the target
    x = 0.0
    y = 0.0
    orientation = 0.0
    for i in range(len(p)):
        x += p[i].x  # add all x of all particles  (if particle close to target then this will work)
        y += p[i].y
        # orientation is tricky because it is cyclic. By normalizing
        # around the first particle we are somewhat more robust to
        # the 0=2pi problem
        orientation += (((p[i].orientation - p[0].orientation + pi) % (2.0 * pi)) 
                        + p[0].orientation - pi)
    return [x / len(p), y / len(p), orientation / len(p)]

# --------
#
# The following code generates the measurements vector
# You can use it to develop your solution.
# 


def generate_ground_truth(motions):  # make robot follow the command vectors of a group of states
                                        # and sense measurement (including noise by default)
    # RETURN:  position wrt each landmark for myrobot  at each move

    myrobot = robot()
    myrobot.set_noise(bearing_noise, steering_noise, distance_noise)

    Z = []
    T = len(motions)

    for t in range(T):
        myrobot = myrobot.move(motions[t])
        Z.append(myrobot.sense())
    #print 'Robot:    ', myrobot
    return [myrobot, Z]

# --------
#
# The following code prints the measurements associated
# with generate_ground_truth
#

def print_measurements(Z):

    T = len(Z)

    print 'measurements = [[%.8s, %.8s, %.8s, %.8s],' % \
        (str(Z[0][0]), str(Z[0][1]), str(Z[0][2]), str(Z[0][3]))
    for t in range(1,T-1):
        print '                [%.8s, %.8s, %.8s, %.8s],' % \
            (str(Z[t][0]), str(Z[t][1]), str(Z[t][2]), str(Z[t][3]))
    print '                [%.8s, %.8s, %.8s, %.8s]]' % \
        (str(Z[T-1][0]), str(Z[T-1][1]), str(Z[T-1][2]), str(Z[T-1][3]))

# --------
#
# The following code checks to see if your particle filter
# localizes the robot to within the desired tolerances
# of the true position. The tolerances are defined at the top.
#

def check_output(final_robot, estimated_position):

    error_x = abs(final_robot.x - estimated_position[0])
    error_y = abs(final_robot.y - estimated_position[1])
    error_orientation = abs(final_robot.orientation - estimated_position[2])
    error_orientation = (error_orientation + pi) % (2.0 * pi) - pi
    correct = error_x < tolerance_xy and error_y < tolerance_xy \
              and error_orientation < tolerance_orientation
    return correct



def particle_filter(motions, measurements, N=500): # I know it's tempting, but don't change N!

     #  motions:  each row is state of myrobot at current move
     #  measurements:  each row is measurement wrt landmarks at current state


    # --------
    #
    # Make particles
    # 

    p = []
    for i in range(N):  # generate N particles
        r = robot()
        r.set_noise(bearing_noise, steering_noise, distance_noise)
        p.append(r)

    # --------
    #
    # Update particles
    #     

    for t in range(len(motions)):  # do this for each motion
    
        # motion update (prediction)
        p2 = []
        for i in range(N):         #(a) move every particle with same movenment of myrobot
            p2.append(p[i].move(motions[t]))
        p = p2

        # measurement update
        w = []
        for i in range(N):         # (b) for each particle calcualte measure of closeness to myrobot
            w.append(p[i].measurement_prob(measurements[t]))

        # resampling
        p3 = []
        index = int(random.random() * N)
        beta = 0.0
        mw = max(w)
        for i in range(N):         # (c) resample N of particles giving higher chance to particles that have higher probability factor
            beta += random.random() * 2.0 * mw
            while beta > w[index]:
                beta -= w[index]
                index = (index + 1) % N
            p3.append(p[index])
        p = p3
    
    return get_position(p)        # (d) return average position of all particles
          
          
          ##  END OF CLASS ROBOT
                 
############## ONLY ADD/MODIFY CODE ABOVE HERE ####################
        



# --------------
# USER INSTRUCTIONS
#
# Now you will put everything together.
#
# First make sure that your sense and move functions
# work as expected for the test cases provided at the
# bottom of the previous two programming assignments.
# Once you are satisfied, copy your sense and move
# definitions into the robot class on this page, BUT
# now include noise.
#
# A good way to include noise in the sense step is to
# add Gaussian noise, centered at zero with variance
# of self.bearing_noise to each bearing. You can do this
# with the command random.gauss(0, self.bearing_noise)
#
# In the move step, you should make sure that your
# actual steering angle is chosen from a Gaussian
# distribution of steering angles. This distribution
# should be centered at the intended steering angle
# with variance of self.steering_noise.
#
# Feel free to use the included set_noise function.
#
# Please do not modify anything except where indicated
# below.

    
    
    ############# ONLY ADD/MODIFY CODE BELOW HERE ###################
       
    # --------
    # move: 
    #   
    
    # copy your code from the previous exercise
    # and modify it so that it simulates motion noise
    # according to the noise parameters
    #           self.steering_noise
    #           self.distance_noise

    # --------
    # sense: 
    #    

    # copy your code from the previous exercise
    # and modify it so that it simulates bearing noise
    # according to
    #           self.bearing_noise

    ############## ONLY ADD/MODIFY CODE ABOVE HERE ####################


## IMPORTANT: You may uncomment the test cases below to test your code.
## But when you submit this code, your test cases MUST be commented
## out.
#motions = [[2. * pi / 10, 20.] for row in range(8)]
##measurements = [[4.746936, 3.859782, 3.045217, 2.045506],
##                [3.510067, 2.916300, 2.146394, 1.598332],
##                [2.972469, 2.407489, 1.588474, 1.611094],
##                [1.906178, 1.193329, 0.619356, 0.807930],
##                [1.352825, 0.662233, 0.144927, 0.799090],
##                [0.856150, 0.214590, 5.651497, 1.062401],
##                [0.194460, 5.660382, 4.761072, 2.471682],
##                [5.717342, 4.736780, 3.909599, 2.342536]]
##
##print particle_filter(motions, measurements)

## 2) You can generate your own test cases by generating
##    measurements using the generate_ground_truth function.
##    It will print the robot's last location when calling it.
##
##
number_of_iterations = 6
motions = [[2. * pi / 20, 12.] for row in range(number_of_iterations)]
##
x = generate_ground_truth(motions)  # x[1] = [z1, z2, z3, z4] at each position
                                    # x[0] = robots state (x,,y,orientataion)
final_robot = x[0]
measurements = x[1]
estimated_position = particle_filter(motions, measurements)
print_measurements(measurements)
print 'Ground truth:    ', final_robot
print 'Particle filter: ', estimated_position
print 'Code check:      ', check_output(final_robot, estimated_position)


