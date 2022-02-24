#!/usr/bin/env python
#coding=utf-8

### find the abosluate path of the description of UR5 Robot
import rospkg
rospack = rospkg.RosPack()
package_path = rospack.get_path("ur_description")

import kdl_parser_py.urdf as kdl_parser
import PyKDL as kdl

from math import pi 
import numpy as np
import sys,os

class ur5(object):
    """
        Using the kdl_parser_py , PyKDL to build A KDL Tree Model
    """
    def __init__(self):
        (status,tree) = kdl_parser.treeFromFile(package_path+"/urdf/ur5_robot.urdf")
        print("\n *** Successfully parsed urdf file and constructed kdl tree ***\
               \n" if status else "Failed to parse urdf file to kdl tree")

        self.tree = tree
        self.chain = tree.getChain("base_link","wrist_3_link")
        # 获取关节数目
        self.num_joints = self.chain.getNrOfJoints()

    
    def __convert2Jnt(self,JntAngles):
        """
            @ Arguments: JntAngles |  list          |  the elements is the angles of the Joints
            @ Output   : Theta     |  KDL::JntArray |  the Length is the number of the Robot's Joints  
        """
        theta = kdl.JntArray(self.num_joints)
        try:
            
            if ( type(JntAngles).__name__ != "list"):
                raise TypeError
            
            if ( len(JntAngles) != self.num_joints):
                raise ValueError
            
            for i in range(len(JntAngles)):
                if(type(JntAngles[i]).__name__ != "float"):
                    raise TypeError
            
            # 将输入角度列表转换为KDL中的数据类型
            theta = kdl.JntArray(self.num_joints)
            for i in range(self.num_joints):
                theta[i] = JntAngles[i]
            return theta

        except TypeError:
            print("\n the type of argument about JntAngles or the type of elements in argument is wrong which should be list and float \n")
            sys.exit(-1)

        except ValueError:
            print("\n the length of argument about JntAngles is wrong \n")
            sys.exit(-1)

    def forwardkinematics(self,JntAngles):
        """
            @ Argument: JntAngles  | List | Unit: rads
            @ Output  : The Pose of End Effector 
            @ Description:  You can get the result of the Robot After called this function 
                            like this: 
                            robot = ur5()
                            eef_pose = robot.forwardkinematics([0.0,0.0,0.0,0.0,0.0,0.0])
                            print(eef_pose.M)  # get the Rotation
                            print(eef_pose.p)  # get the position
        """
        
        fk_pos_solver = kdl.ChainFkSolverPos_recursive(self.chain)
        end_effector_pose = kdl.Frame()
        theta = self.__convert2Jnt(JntAngles)
        fk_pos_solver.JntToCart(theta,end_effector_pose)

        return end_effector_pose
    
    def getJacobian(self,JntAngles):
        """
            @ Argument: JntAngles | List | Unit: rads
            @ Output  : The Jacobian Of the Robot when it's Joint is set to the JntAngles
            @ Description:  Needs to know the type of output , J , is kdl::Jacobain,
                            So You May need to convert the output to the another type You want. 
        """
        
        # 实例化Jacobian求解器
        jacobian_solver = kdl.ChainJntToJacSolver(self.chain)
        J = kdl.Jacobian(self.num_joints)
        theta = self.__convert2Jnt(JntAngles)
        
        # 计算Jacobian矩阵
        jacobian_solver.JntToJac(theta,J)

        return J

    def inversekinematics(self,End_Effector_pose,JntArray_Init=[0.0,0.0,0.0,0.0,0.0,0.0]):
        """
            @ Arguments: JntArray_Init      List          the Initialization Joint Pose of the Robot 
                         End_Effector_pose  KDL::Frame    the Desired End_Effector Pose          
        """

        try:
            if (isinstance(End_Effector_pose,kdl.Frame)):
                # set the Limits 
                theta_min = kdl.JntArray(self.num_joints)
                theta_max = kdl.JntArray(self.num_joints)
                theta_min[0] = theta_min[1] = theta_min[2] = theta_min[3] = theta_min[4] = theta_min[5] = -pi
                theta_max[0] = theta_max[1] = theta_max[2] = theta_max[3] = theta_max[4] = theta_max[5] = pi
        
                # set the init theta and output theta
                theta_init = kdl.JntArray(self.num_joints)
                theta_out = kdl.JntArray(self.num_joints)
        
                # the IKSolverPos_NR_JL will calculate the IK with FK_pos algorithm and ik_vel algorithem
                theta_init = self.__convert2Jnt(JntArray_Init)
                ik_vel_solver = kdl.ChainIkSolverVel_pinv(self.chain)
                fk_pos_solver = kdl.ChainFkSolverPos_recursive(self.chain)
        
                ik_pos_solver = kdl.ChainIkSolverPos_NR_JL(self.chain,theta_min,theta_max,fk_pos_solver,ik_vel_solver)
                desired_position = kdl.Frame(End_Effector_pose.M,End_Effector_pose.p)
        
        
                ik_pos_solver.CartToJnt(theta_init,desired_position,theta_out)
        
                return theta_out
            else:
                raise TypeError

        except TypeError:
            print(" The type of End-Effector-pose is wrong")
            sys.exit(-1)
    

    ## robotics velocity parts

    def fkvelkinematics(self,theta,theta_delta):
        """
            @ Arguments : theta       List  the current joint angles of Robot
                          theta_delta List  the $ \delta theta $ , small increase parts or decrease parts   

            @ Outputs   : twist       KDL::Twist    
        """
        try:
            if (len(theta) != self.num_joints or len(theta_delta) != self.num_joints):
                raise ValueError
            fk_vel_solver = kdl.ChainFkSolverVel_recursive(self.chain)
            twist = kdl.FrameVel()
            
            
            theta_ = self.__convert2Jnt(theta)
            theta_delta_ = self.__convert2Jnt(theta_delta)
            
            fk_vel_solver.JntToCart( kdl.JntArrayVel(theta_, theta_delta_) , twist )

            return twist.deriv()

        except ValueError:
            print(" the length of theta and theta_delta is wrong which should be same as the number of Joints In the Robot .")
            sys.exit(-1)        

    def ikvelkinematics(self,theta,twist):
        """
        """
        print(len(theta)!=self.num_joints)
        print(len(twist)!=6)
        try: 
            if (len(theta) != self.num_joints or len(twist) != 6):
                raise ValueError
            
            ### Calculated Joint Velocity Ik by mapping Jacobian
            J = self.getJacobian(theta)
            print(J)
            J = self.__convert2mat(J)
            twist = np.array(twist,dtype=np.float)
            print(twist.shape)
            print(J)

            J_pinv = np.linalg.pinv(J)
            print(J_pinv.shape)
            
            theta_delta = np.dot(twist,J_pinv)

            return theta_delta

            ### Actually there are the other way to calculate the Velocity IK , which is figuring out with KDL solver
            ### theta_dot_out = kdl.JntArray(self.num_joints)
            ### ik_vel_solver.CartToJnt(theta,twist.deriv(),theta_dot_out)

        except ValueError:
            print("\n *** the length of the input arguments may not correct *** \n")
            sys.exit(-1)
    
    def __convert2mat(self,J):
        """
            @ Argument: J  kdl.Jacobian 
            @ Output  : Jac  numpy.array     shape like the input argument J 
        """
        Jac = np.zeros((J.rows(),J.columns()))
        for i in range(J.rows()):
            for j in range(J.columns()):
                ## In PyKDL we should iter Jacobian matrix like this
                Jac[i][j] = J[i,j]
        
        return Jac

    ## Dynamics Parts
    def gravityTermCal(self,JntArray):

        dynamic_param_solver = kdl.ChainDynParam(self.chain,kdl.Vector(0,0,-9.0))
        angles = self.__convert2Jnt(JntArray)
        G_vector = kdl.JntArray(self.num_joints)

        dynamic_param_solver.JntToGravity(angles,G_vector)

        return G_vector

    def tauCalcu(self,theta,theta_dot,theta_dotdot):

        # Initialize dynamics parameters
        M = kdl.JntSpaceInertiaMatrix(self.num_joints)
        C = kdl.JntArray(self.num_joints)
        G = kdl.JntArray(self.num_joints)

        theta_init = self.__convert2Jnt(theta)
        theta_dot = self.__convert2Jnt(theta_dot)
        dynamic_param_solver = kdl.ChainDynParam(self.chain,kdl.Vector(0,0,-9.8))

        # Calculate dynamics parameter and convert them to matrix to manipulate
        dynamic_param_solver.JntToMass(theta_init,M)
        dynamic_param_solver.JntToCoriolis(theta_init,theta_dot,C)       
        dynamic_param_solver.JntToGravity(theta_init,G)

        M = self.__convert2mat(M)
        C = self.__convert2array(C)
        G = self.__convert2array(G)
        # Calculate the required torque from dynamic equation
        theta_dotdot = np.array(theta_dotdot)
        tau = np.dot(M,theta_dotdot) + C + G

        return tau
    
    def __convert2array(self,JntArray):

        array = np.zeros((self.num_joints,))
        for i in range(self.num_joints):
            array[i] = JntArray[i]
        
        return array
