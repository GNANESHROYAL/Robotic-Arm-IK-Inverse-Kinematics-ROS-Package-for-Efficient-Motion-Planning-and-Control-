#!/usr/bin/env python3
import numpy as np
import rospy
from urdf_parser_py.urdf import URDF
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

def joint_position():
    rospy.init_node('ik',anonymous=False)
    pub =rospy.Publisher('/joint_states',JointState, queue_size=10)
    rate = rospy.Rate(10)
    my_param=rospy.get_param('robot_description')
    robot=URDF.from_xml_string(my_param)
    j_n=[joint.name for joint in robot.joints]
    j_o=[joint.origin for joint in robot.joints]
    n_l=len(robot.link_map)
    joints=len(robot.joints)-3
    print(joints)
    xc=0
    yc=0
    zc=0
    thetas=np.zeros(joints)
    dist=[]
    for i in range(1,len(j_n)):
        o1 =j_o[i-1]
        o2 =j_o[i]
        di=np.sqrt(pow((o1.xyz[0]-o2.xyz[0]),2)+pow((o1.xyz[1]-o2.xyz[1]),2)+pow((o1.xyz[2]-o2.xyz[2]),2))
        dist.append(di)
    dist=np.array(dist)
    dist=dist.astype('float32')
    print(len(dist))
    co=sum(dist)
    j_a_r=[]
    del j_n[0]
    for joint in robot.joints:
        if joint.type == "revolute":
            j_l=np.linspace(joint.limit.lower,joint.limit.upper,100)
            j_a_r.append(j_l)
    print("limits x:({},{}) y: ({},{}) z:({},{})".format(-co,co,-co,co,-co,0))
    print("current origin: ",xc,yc,zc)
    while True:
        x,y,z=input("Enter coordinates: ").split()
        for i in range(0,2):
            def jacob(joints,thetas):
                j=np.zeros((3,len(dist)))
                co_mat=np.zeros((n_l,n_l))
                trig=np.zeros(len(dist))
                print(len(trig))
                trig1=np.zeros(len(dist))
                trig2=np.zeros(len(dist))
                print(len(dist))
                
                for i in range(joints):
                    trig[i]=-np.sin(np.sum(thetas[:i+1]))
                    trig1[i]=np.cos(np.sum(thetas[:i+1]))
                    trig2[i]=np.sin(np.sum(thetas[:i+1]))
                j[0,:]=dist*trig
                j[1,:]=dist*trig1
                j[2,:]=dist*trig2
                j=np.linalg.pinv(j)
                error=[xd-xc,yd-yc,zd-zc]
                error=np.array(error)
                error=error.reshape(3,1)
                thetas=np.dot(j,error)
                for i in range(n_l):
                    for j in range(i+1,n_l):
                        if np.linalg.norm([xd,yd,zd])<0.1:
                            co_mat[i,j]=1
                            print(co_mat)
                return thetas
            xd=float(x)
            yd=float(y)
            zd=float(z)
            if -co<=xd<=co and -co<=yd<=co and -co<=zd<=0 :
                k=jacob(joints,thetas)
                thetas=(k*180)/3.14
                xc=xd
                yc=yd
                zc=zd
                if i==1:
                    print("limits x:({},{}) y: ({},{}) z:({},{})".format(-co,co,-co,co,-co,0))
                    print("current origin: ",xc,yc,zc)
                elif i==0:
                    k=k.astype('Float64')
                    j_s=JointState()
                    j_s.position=k
                    j_s.name=j_n
                    j_s.header.stamp =rospy.Time.now()
                    j_s.header.frame_id="mamulga_undadhu"
                    print(thetas)
                    rospy.loginfo(j_s)
                    pub.publish(j_s)
                    rate.sleep()
            else:
                print("Error input!!!")
                continue

if __name__ == '__main__':
   try:
       joint_position()
   except rospy.ROSInterruptException:
        pass