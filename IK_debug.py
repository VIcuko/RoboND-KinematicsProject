from sympy import *
from time import time
from mpmath import radians
import math
import tf

'''
Format of test case is [ [[EE position],[EE orientation as quaternions]],[WC location],[joint angles]]
You can generate additional test cases by setting up your kuka project and running `$ roslaunch kuka_arm forward_kinematics.launch`
From here you can adjust the joint angles to find thetas, use the gripper to extract positions and orientation (in quaternion xyzw) and lastly use link 5
to find the position of the wrist center. These newly generated test cases can be added to the test_cases dictionary.
'''

test_cases = {1:[[[2.16135,-1.42635,1.55109],
                  [0.708611,0.186356,-0.157931,0.661967]],
                  [1.89451,-1.44302,1.69366],
                  [-0.65,0.45,-0.36,0.95,0.79,0.49]],
              2:[[[-0.56754,0.93663,3.0038],
                  [0.62073, 0.48318,0.38759,0.480629]],
                  [-0.638,0.64198,2.9988],
                  [-0.79,-0.11,-2.33,1.94,1.14,-3.68]],
              3:[[[-1.3863,0.02074,0.90986],
                  [0.01735,-0.2179,0.9025,0.371016]],
                  [-1.1669,-0.17989,0.85137],
                  [-2.99,-0.12,0.94,4.06,1.29,-4.12]],
              4:[],
              5:[]}


def test_code(test_case):
    ## Set up code
    ## Do not modify!
    x = 0
    class Position:
        def __init__(self,EE_pos):
            self.x = EE_pos[0]
            self.y = EE_pos[1]
            self.z = EE_pos[2]
    class Orientation:
        def __init__(self,EE_ori):
            self.x = EE_ori[0]
            self.y = EE_ori[1]
            self.z = EE_ori[2]
            self.w = EE_ori[3]

    position = Position(test_case[0][0])
    orientation = Orientation(test_case[0][1])

    class Combine:
        def __init__(self,position,orientation):
            self.position = position
            self.orientation = orientation

    comb = Combine(position,orientation)

    class Pose:
        def __init__(self,comb):
            self.poses = [comb]

    req = Pose(comb)
    start_time = time()
    
    ########################################################################################
    ## 

    ## Insert IK code here!
    
    ### FK code here
    # First we create simols (bare in mind the Kuka KR210 has 7 elements):

    # theta
    q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
    # alpha
    alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
    #distances between joints axes
    d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
    a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
    

    # DH parameters
    s = {alpha0: 0,     a0:   0,    d1: 0.75,    q1: q1,
         alpha1: -pi/2, a1: 0.35,   d2: 0,       q2: q2 -pi/2,  
         alpha2: 0,     a2: 1.25,   d3: 0,       q3: q3,
         alpha3: -pi/2, a3: -0.054, d4: 1.5,     q4: q4,
         alpha4: pi/2,  a4:   0,    d5: 0,       q5: q5,
         alpha5: -pi/2, a5:   0,    d6: 0,       q6: q6,
         alpha6: 0,     a6:   0,    d7: 0.303,   q7: 0}

    # Modified DH Transformation matrix
    def DH_T_Matrix (q, alpha, d, a):
        DH_Matrix = Matrix([[             cos(q),          -sin(q),           0,             a],
                            [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                            [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                            [                 0,                 0,           0,             1]])
        return DH_Matrix

# Create individual transformation matrices

    T0_1 = DH_T_Matrix(q1, alpha0, d1, a0).subs(s)
    T1_2 = DH_T_Matrix(q2, alpha1, d2, a1).subs(s)
    T2_3 = DH_T_Matrix(q3, alpha2, d3, a2).subs(s)
    T3_4 = DH_T_Matrix(q4, alpha3, d4, a3).subs(s)
    T4_5 = DH_T_Matrix(q5, alpha4, d5, a4).subs(s)
    T5_6 = DH_T_Matrix(q6, alpha5, d6, a5).subs(s)
    T6_G = DH_T_Matrix(q7, alpha6, d7, a6).subs(s)

# Extract rotation matrices from the transformation matrices
    #R0_1 = T0_1[0:3,0:3]
    #R1_2 = T1_2[0:3,0:3]
    #R2_3 = T2_3[0:3,0:3]
    #R3_4 = T3_4[0:3,0:3]
    #R4_5 = T4_5[0:3,0:3]
    #R5_6 = T5_6[0:3,0:3]
    #R6_G = T6_G[0:3,0:3]

# Now the calculations from baselink to all points:

    #T0_2 = simplify(T0_1 * T1_2)
    #T0_3 = simplify(T0_2 * T2_3)
    #T0_4 = simplify(T0_3 * T3_4)
    #T0_5 = simplify(T0_4 * T4_5)
    #T0_6 = simplify(T0_5 * T5_6)
    #T0_G = simplify(T0_1*T1_2*T2_3*T3_4*T4_5*T5_6*T6_G)
   
    T0_G = Matrix([[((sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) + sin(q5)*cos(q1)*cos(q2 + q3))*cos(q6) + (sin(q1)*cos(q4) - sin(q4)*sin(q2 + q3)*cos(q1))*sin(q6), -((sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) + sin(q5)*cos(q1)*cos(q2 + q3))*sin(q6) + (sin(q1)*cos(q4) - sin(q4)*sin(q2 + q3)*cos(q1))*cos(q6), -(sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*sin(q5) + cos(q1)*cos(q5)*cos(q2 + q3), -0.303*sin(q1)*sin(q4)*sin(q5) + 1.25*sin(q2)*cos(q1) - 0.303*sin(q5)*sin(q2 + q3)*cos(q1)*cos(q4) - 0.054*sin(q2 + q3)*cos(q1) + 0.303*cos(q1)*cos(q5)*cos(q2 + q3) + 1.5*cos(q1)*cos(q2 + q3) + 0.35*cos(q1)],
                   [((sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*cos(q5) + sin(q1)*sin(q5)*cos(q2 + q3))*cos(q6) - (sin(q1)*sin(q4)*sin(q2 + q3) + cos(q1)*cos(q4))*sin(q6), -((sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*cos(q5) + sin(q1)*sin(q5)*cos(q2 + q3))*sin(q6) - (sin(q1)*sin(q4)*sin(q2 + q3) + cos(q1)*cos(q4))*cos(q6), -(sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*sin(q5) + sin(q1)*cos(q5)*cos(q2 + q3),  1.25*sin(q1)*sin(q2) - 0.303*sin(q1)*sin(q5)*sin(q2 + q3)*cos(q4) - 0.054*sin(q1)*sin(q2 + q3) + 0.303*sin(q1)*cos(q5)*cos(q2 + q3) + 1.5*sin(q1)*cos(q2 + q3) + 0.35*sin(q1) + 0.303*sin(q4)*sin(q5)*cos(q1)],
                   [                                                               -(sin(q5)*sin(q2 + q3) - cos(q4)*cos(q5)*cos(q2 + q3))*cos(q6) - sin(q4)*sin(q6)*cos(q2 + q3),                                                                  (sin(q5)*sin(q2 + q3) - cos(q4)*cos(q5)*cos(q2 + q3))*sin(q6) - sin(q4)*cos(q6)*cos(q2 + q3),                                     -sin(q5)*cos(q4)*cos(q2 + q3) - sin(q2 + q3)*cos(q5),                                                                                 -0.303*sin(q5)*cos(q4)*cos(q2 + q3) - 0.303*sin(q2 + q3)*cos(q5) - 1.5*sin(q2 + q3) + 1.25*cos(q2) - 0.054*cos(q2 + q3) + 0.75],
                   [                                                                                                                                                           0,                                                                                                                                                             0,                                                                                        0,                                                                                                                                                                                                              1]])
    #R0_2 = simplify(R0_1 * R1_2)
    #R0_3 = simplify(R0_2 * R2_3)
    R0_3 = Matrix([[sin(q2 + q3)*cos(q1), cos(q1)*cos(q2 + q3), -sin(q1)],
                   [sin(q1)*sin(q2 + q3), sin(q1)*cos(q2 + q3),  cos(q1)],
                   [        cos(q2 + q3),        -sin(q2 + q3),        0]])

    #R0_4 = simplify(R0_3 * R3_4)
    #R0_5 = simplify(R0_4 * R4_5)
    #R0_6 = simplify(R0_5 * R5_6)
    #R0_G = simplify(R0_6 * R6_G)

    ###

    # IK code starts here
    for x in xrange(0, len(req.poses)):
    # Extract end-effector position and orientation from request
    # px,py,pz = end-effector position
    # roll, pitch, yaw = end-effector orientation
        px = req.poses[x].position.x
        py = req.poses[x].position.y
        pz = req.poses[x].position.z

        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
            [req.poses[x].orientation.x, req.poses[x].orientation.y,
                req.poses[x].orientation.z, req.poses[x].orientation.w])
     
            ### Your IK code here 
        # Compensate for rotation discrepancy between DH parameters and Gazebo
        
        r, p, y = symbols('r p y')

        #R_z = Matrix([[ cos(y), -sin(y),   0],
        #              [ sin(y),  cos(y),   0],
        #              [      0,       0,   1]])

        #R_y = Matrix([[ cos(p),    0,   sin(p)],
        #              [      0,    1,        0],
        #              [-sin(p),    0,  cos(p)]])

        #R_x = Matrix([[ 1,         0,       0],
        #              [ 0,    cos(r), -sin(r)],
        #              [ 0,    sin(r),  cos(r)]])

        #R_G = R_z * R_y * R_x
        R_G = Matrix([[cos(p)*cos(y), sin(p)*sin(r)*cos(y) - sin(y)*cos(r), sin(p)*cos(r)*cos(y) + sin(r)*sin(y)],
                      [sin(y)*cos(p), sin(p)*sin(r)*sin(y) + cos(r)*cos(y), sin(p)*sin(y)*cos(r) - sin(r)*cos(y)],
                      [      -sin(p),                        sin(r)*cos(p),                        cos(p)*cos(r)]])
        
        #R_corr = simplify(R_z.subs(y, pi) * R_y.subs(p, -pi/2)
        R_corr = Matrix([[-6.12323399573677e-17, -1.22464679914735e-16,                   1.0],
                         [ 7.49879891330929e-33,                  -1.0, -1.22464679914735e-16],
                         [                  1.0,                     0,  6.12323399573677e-17]])

        R_G = R_G * R_corr
        R_G = R_G.subs({'r': roll, 'p': pitch, 'y': yaw})

        #End effector position obtained previously into matrix for further use
        G_pos = Matrix([[px],
                        [py],
                        [pz]])
        #Wrist position 
        Wpos = G_pos - 0.303 * R_G[:,2]
        # Calculate joint angles using Geometric IK method
        
        theta1 = atan2(Wpos[1],Wpos[0])
        
        #Triangle for theta2 and 3
        side_a = 1.5
        side_b = sqrt(pow((sqrt(Wpos[0]*Wpos[0] + Wpos[1]*Wpos[1]) - 0.35),2) + pow((Wpos[2] - 0.75),2))
        side_c = 1.25
        
        angle_a = acos((side_b*side_b + side_c*side_c - side_a*side_a) / (2*side_b*side_c))
        angle_b = acos((side_a*side_a + side_c*side_c - side_b*side_b) / (2*side_a*side_c))
        angle_c = acos((side_a*side_a + side_b*side_b - side_c*side_c) / (2*side_a*side_b))

        theta2 = pi/2 - angle_a - atan2(Wpos[2] - 0.75, sqrt(Wpos[0]*Wpos[0] + Wpos[1]*Wpos[1]) - 0.35)
        theta3 = pi/2 - (angle_b + 0.036)

        #Now we use the extracted rotation matrix from link 0 to 3:
        #And introduce the angle values
        R0_3 = R0_3.evalf(subs={q1:theta1, q2:theta2, q3:theta3})

        #Now we calculate the rotation matrix from link 3 to 6 (using LU decomposition)

        R3_6 = simplify (R0_3.inv("LU") * R_G)

        #Now we can calculate the remaining thetas:

        theta4 = atan2(R3_6[2,2], -R3_6[0,2])
        theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]),R3_6[1,2])
        theta6 = atan2(-R3_6[1,1], R3_6[1,0])

    ## 
    ########################################################################################
    
    ########################################################################################
    ## For additional debugging add your forward kinematics here. Use your previously calculated thetas
    ## as the input and output the position of your end effector as your_ee = [x,y,z]

    ## (OPTIONAL) YOUR CODE HERE!

    For_Kin = T0_G.evalf(subs={q1:theta1, q2:theta2, q3:theta3, q4:theta4, q5:theta5, q6:theta6})

    ## End your code input for forward kinematics here!
    ########################################################################################

    ## For error analysis please set the following variables of your WC location and EE location in the format of [x,y,z]
    your_wc = [Wpos[0],Wpos[1], Wpos[2]] # <--- Load your calculated WC values in this array
    your_ee = [For_Kin[0,3],For_Kin[1,3], For_Kin[2,3]] # <--- Load your calculated end effector value from your forward kinematics
    ########################################################################################

    ## Error analysis
    print ("\nTotal run time to calculate joint angles from pose is %04.4f seconds" % (time()-start_time))

    # Find WC error
    if not(sum(your_wc)==3):
        print ("your_wc:",your_wc)
        print ("test_case:",test_case)
        wc_x_e = abs(your_wc[0]-test_case[1][0])
        wc_y_e = abs(your_wc[1]-test_case[1][1])
        wc_z_e = abs(your_wc[2]-test_case[1][2])

        wc_offset = sqrt(wc_x_e**2 + wc_y_e**2 + wc_z_e**2)
        print ("\nWrist error for x position is: %04.8f" % wc_x_e)
        print ("Wrist error for y position is: %04.8f" % wc_y_e)
        print ("Wrist error for z position is: %04.8f" % wc_z_e)
        print ("Overall wrist offset is: %04.8f units" % wc_offset)

    # Find theta errors
    t_1_e = abs(theta1-test_case[2][0])
    t_2_e = abs(theta2-test_case[2][1])
    t_3_e = abs(theta3-test_case[2][2])
    t_4_e = abs(theta4-test_case[2][3])
    t_5_e = abs(theta5-test_case[2][4])
    t_6_e = abs(theta6-test_case[2][5])
    print ("\nTheta 1 error is: %04.8f" % t_1_e)
    print ("Theta 2 error is: %04.8f" % t_2_e)
    print ("Theta 3 error is: %04.8f" % t_3_e)
    print ("Theta 4 error is: %04.8f" % t_4_e)
    print ("Theta 5 error is: %04.8f" % t_5_e)
    print ("Theta 6 error is: %04.8f" % t_6_e)
    print ("\n**These theta errors may not be a correct representation of your code, due to the fact \
           \nthat the arm can have muliple positions. It is best to add your forward kinmeatics to \
           \nconfirm whether your code is working or not**")
    print (" ")

    # Find FK EE error
    if not(sum(your_ee)==3):
        ee_x_e = abs(your_ee[0]-test_case[0][0][0])
        ee_y_e = abs(your_ee[1]-test_case[0][0][1])
        ee_z_e = abs(your_ee[2]-test_case[0][0][2])
        ee_offset = sqrt(ee_x_e**2 + ee_y_e**2 + ee_z_e**2)
        print ("\nEnd effector error for x position is: %04.8f" % ee_x_e)
        print ("End effector error for y position is: %04.8f" % ee_y_e)
        print ("End effector error for z position is: %04.8f" % ee_z_e)
        print ("Overall end effector offset is: %04.8f units \n" % ee_offset)




if __name__ == "__main__":
    # Change test case number for different scenarios
    test_case_number = 1

    test_code(test_cases[test_case_number])
