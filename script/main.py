#!/usr/bin/env python
#coding=utf-8


from robot import ur5
from math import pi


def main():
    robot = ur5()
    pose = robot.forwardkinematics([0.0,0.0,0.0,0.0,0.0,0.0])
    print("\n *** The Pose of End Effector *** \n")
    print(pose)

    pose1 = robot.forwardkinematics([0.0,pi/2,pi/2,pi/4,0.0,0.0])
    print("\n *** IK test Position *** \n")
    print(pose1)

    Jaco = robot.getJacobian([0.0,0.0,0.0,pi/4,0.0,0.1])
    print("\n *** the jacobian of Robot *** \n")
    print(Jaco)

    theta_out = robot.inversekinematics(pose1)
    print("\n *** the Output of IK *** \n")
    print(theta_out)

    theta = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    theta_delta = [0.0, 0.0, 0.0, 0.5, 0.0, 0.0]
    twist_deriv = robot.fkvelkinematics( theta, theta_delta)
    print("\n *** the twist is *** \n")
    print(twist_deriv)
    print(type(twist_deriv))

    twist = [-0.047325,0.0,0.0,0.0,0.5,0.0]
    theta_out = robot.ikvelkinematics(theta,twist)
    print("\n *** the Inverse Velocity IK  Output ***\n")
    print(theta_out)

    theta = [0.0,1.44,0.0,0.0,0.0,0.0]
    G_term = robot.gravityTermCal(theta)
    print("\n *** G_term in the current Joint_Angle *** \n")
    print(G_term)
    print("\n")

    theta_dot = [0.0,0.01,0.0,0.0,0.0,0.0]
    theta_dotdot = [0.0, 0.01, 0.0,0.0,0.0,0.0 ]
    tau = robot.tauCalcu(theta,theta_dot,theta_dotdot)
    print(tau)

if __name__ == "__main__":
    main()