import numpy as np
from math import pi

'''
    Class to calculate the inverse kinematics for the stewart platform.
    Needs pose and twist input to calculate leg length and velocity
    All length-units is written in meters [m]
'''
class InverseKinematics(object):
    def __init__(self):
        
        # minimum possible position in z (heave) direction
        self.__z_min = 0.16107

        # defining base plate position vectors
        self.__a1 = np.array([
            [-0.14228], [-0.0475], [0.]
        ])

        self.__a2 = np.array([
            [-0.11228], [-0.09947], [0.]
        ])

        self.__a3 = np.array([
            [0.11228], [-0.09947], [0.]
        ])

        self.__a4 = np.array([
            [0.14228], [-0.0475], [0.]
        ])

        self.__a5 = np.array([
            [0.030], [0.14697], [0.]
        ])

        self.__a6 = np.array([
            [-0.030], [0.14697], [0.]
        ])

        # defining tool plate position vectors
        self.__b1 = np.array([
            [-0.09761], [0.02172], [0.]
        ])

        self.__b2 = np.array([
            [-0.030], [-0.09539], [0.]
        ])

        self.__b3 = np.array([
            [0.030], [-0.09539], [0.]
        ])

        self.__b4 = np.array([
            [0.09761], [0.02172], [0.]
        ])

        self.__b5 = np.array([
            [0.06761], [0.07368], [0.]
        ])

        self.__b6 = np.array([
            [-0.06761], [0.07368], [0.]
        ])
    

    def __jacobian(self):
        
        # unit-vectors of s (leg-vectors)
        e_ = np.array([  self.__s1 / self.__l1,
                        self.__s2 / self.__l2,
                        self.__s3 / self.__l3,
                        self.__s4 / self.__l4,
                        self.__s5 / self.__l5,
                        self.__s6 / self.__l6,  ])
        
        # rotation matrix times tool-plate position vectors
        rot_ = np.array([    self.__R @ self.__b1,
                            self.__R @ self.__b2,
                            self.__R @ self.__b3,
                            self.__R @ self.__b4,
                            self.__R @ self.__b5,
                            self.__R @ self.__b6    ])


        rot = rot_.reshape(6, 1, 3)
        e = e_.reshape(6, 1, 3)

        # cross-product
        cross1 = np.cross(rot[0], e[0])
        cross2 = np.cross(rot[1], e[1])
        cross3 = np.cross(rot[2], e[2])
        cross4 = np.cross(rot[3], e[3])
        cross5 = np.cross(rot[4], e[4])
        cross6 = np.cross(rot[5], e[5])

        # add together to a single array per row of the jacobian
        # where the unit vector represents the translational part
        # and the cross product represents the rotational part of the jacobian
        J1 = np.hstack((e[0], cross1))
        J2 = np.hstack((e[1], cross2))
        J3 = np.hstack((e[2], cross3))
        J4 = np.hstack((e[3], cross4))
        J5 = np.hstack((e[4], cross5))
        J6 = np.hstack((e[5], cross6))

        # put all the rows above together in a single 6x6 matrix
        J = np.concatenate((J1, J2, J3, J4, J5, J6), axis=0)

        return J


    def calc_output(self, pose, twist):

        x = pose[0]
        y = pose[1]
        z = self.__z_min + pose[2]

        # calculating sin and cos values for matrices
        phi_sin = np.sin(pose[3])   # roll
        phi_cos = np.cos(pose[3])

        theta_sin = np.sin(pose[4]) # pitch
        theta_cos = np.cos(pose[4])

        psi_sin = np.sin(pose[5])   # yaw
        psi_cos = np.cos(pose[5])

        # defining the rotation matrices for each axis of rotation
        r_x = np.array([
            [1., 0., 0.],
            [0., phi_cos, -phi_sin],
            [0., phi_sin, phi_cos]
        ])

        r_y = np.array([
            [theta_cos, 0., theta_sin],
            [0., 1., 0.],
            [-theta_sin, 0., theta_cos]
        ])

        r_z = np.array([
            [psi_cos, -psi_sin, 0.],
            [psi_sin, psi_cos, 0.],
            [0., 0., 1.]
        ])

        # defining total rotation matrix
        self.__R = r_z @ r_y @ r_x

        # defining position vector
        p = np.array([
            [x],
            [y],
            [z]
        ])

        # calculating leg-vectors
        self.__s1 = p + (self.__R @ self.__b1) - self.__a1
        self.__s2 = p + (self.__R @ self.__b2) - self.__a2
        self.__s3 = p + (self.__R @ self.__b3) - self.__a3
        self.__s4 = p + (self.__R @ self.__b4) - self.__a4
        self.__s5 = p + (self.__R @ self.__b5) - self.__a5
        self.__s6 = p + (self.__R @ self.__b6) - self.__a6

        # calculating leg lengths (leg-vector magnitude)
        self.__l1 = np.sqrt(np.float_power(self.__s1[0, 0], 2) + np.float_power(self.__s1[1, 0], 2) + np.float_power(self.__s1[2, 0], 2))
        self.__l2 = np.sqrt(np.float_power(self.__s2[0, 0], 2) + np.float_power(self.__s2[1, 0], 2) + np.float_power(self.__s2[2, 0], 2))
        self.__l3 = np.sqrt(np.float_power(self.__s3[0, 0], 2) + np.float_power(self.__s3[1, 0], 2) + np.float_power(self.__s3[2, 0], 2))
        self.__l4 = np.sqrt(np.float_power(self.__s4[0, 0], 2) + np.float_power(self.__s4[1, 0], 2) + np.float_power(self.__s4[2, 0], 2))
        self.__l5 = np.sqrt(np.float_power(self.__s5[0, 0], 2) + np.float_power(self.__s5[1, 0], 2) + np.float_power(self.__s5[2, 0], 2))
        self.__l6 = np.sqrt(np.float_power(self.__s6[0, 0], 2) + np.float_power(self.__s6[1, 0], 2) + np.float_power(self.__s6[2, 0], 2))

        # actuator stroke position
        d1 = self.__l1 - 0.181
        d2 = self.__l2 - 0.181
        d3 = self.__l3 - 0.181
        d4 = self.__l4 - 0.181
        d5 = self.__l5 - 0.181
        d6 = self.__l6 - 0.181

        # inverse jacobian with respect to q (pose params, stored above as private self variables)
        J = self.__jacobian()

        # actuator stroke velocity
        d_dot = np.hstack((J @ twist))

        return np.array([[d1, d2, d3, d4, d5, d6], d_dot])

        
