import numpy as np
import rospy
import tf
import tf2_ros
from std_msgs.msg import Header
import geometry_msgs.msg

from scaler_kin.v3.SCALER_v2_Leg_6DOF_gripper import Leg
from scaler_kin.v3.SCALAR_kinematics import scalar_k


def publish_to_tf(T,parent_name, child_name):
    ob = geometry_msgs.msg.TransformStamped()
    ob.header.stamp = rospy.Time.now()
    ob.header.frame_id = parent_name
    ob.child_frame_id = child_name
    position = tf.transformations.translation_from_matrix(T)
    orientation = tf.transformations.quaternion_from_matrix(T)
    ob.transform.translation.x = position[0]
    ob.transform.translation.y = position[1]
    ob.transform.translation.z = position[2]
    ob.transform.rotation.x = orientation[0]
    ob.transform.rotation.y = orientation[1]
    ob.transform.rotation.z = orientation[2]
    ob.transform.rotation.w = orientation[3]
    return ob






if __name__ == '__main__':


    rospy.init_node('tf_pub')
    br = tf2_ros.TransformBroadcaster()
    T_0 = tf.transformations.quaternion_about_axis(np.pi, (0.0, 0.0, 1.0))
    T_0 = tf.transformations.quaternion_matrix(T_0)
    myLeg = Leg()
    rate = rospy.Rate(100)
    shoulder_angle = 0.0*np.pi
    q1 = 0.0*np.pi
    q2 = 0.5*np.pi
    th4 =0/180.0*np.pi
    th5 =0/180.0*np.pi
    th6 =0/180.0*np.pi
    
    L_actuator = 105
    theta_actuator = 0
    my_scalar_k = scalar_k()
    while 1:
        for i in range(4):
            br.sendTransform(publish_to_tf(T_0,"base_link", "Bpysilvia"))
            T_0_shi = myLeg.leg_fk_direct_calculation(0.0, [shoulder_angle,q1,q2,th4,th5,th6], i, 1, use_quaternion = False)
            T_0_shi[0:3,3] = T_0_shi[0:3,3] / 1000.0
            br.sendTransform(publish_to_tf(T_0_shi,"Bpysilvia", "sh"+str(i+1)))
        
            T_0_A = myLeg.leg_fk_direct_calculation(0.0, [shoulder_angle,q1,q2,th4,th5,th6], i, 2, use_quaternion = False)
            T_0_A[0:3,3] = T_0_A[0:3,3] / 1000.0
            br.sendTransform(publish_to_tf(T_0_A,"Bpysilvia", "A"+str(i+1)))

            T_0_F = myLeg.leg_fk_direct_calculation(0.0, [shoulder_angle,q1,q2,th4,th5,th6], i, 3, use_quaternion = False)
            T_0_F[0:3,3] = T_0_F[0:3,3] / 1000.0
            br.sendTransform(publish_to_tf(T_0_F,"Bpysilvia", "F"+str(i+1)))

            T_0_B = myLeg.leg_fk_direct_calculation(0.0, [shoulder_angle,q1,q2,th4,th5,th6], i, 4, use_quaternion = False)
            T_0_B[0:3,3] = T_0_B[0:3,3] / 1000.0
            br.sendTransform(publish_to_tf(T_0_B,"Bpysilvia", "B"+str(i+1)))

            T_0_E = myLeg.leg_fk_direct_calculation(0.0, [shoulder_angle,q1,q2,th4,th5,th6], i, 5, use_quaternion = False)
            T_0_E[0:3,3] = T_0_E[0:3,3] / 1000.0
            br.sendTransform(publish_to_tf(T_0_E,"Bpysilvia", "E"+str(i+1)))

            T_0_C = myLeg.leg_fk_direct_calculation(0.0, [shoulder_angle,q1,q2,th4,th5,th6], i, 6, use_quaternion = False)
            T_0_C[0:3,3] = T_0_C[0:3,3] / 1000.0
            br.sendTransform(publish_to_tf(T_0_C,"Bpysilvia", "C"+str(i+1)))

            T_0_wrist1 = myLeg.leg_fk_direct_calculation(0.0, [shoulder_angle,q1,q2,th4,th5,th6], i, 7, use_quaternion = False)
            T_0_wrist1[0:3,3] = T_0_wrist1[0:3,3] / 1000.0
            br.sendTransform(publish_to_tf(T_0_wrist1,"Bpysilvia", "wrist1"+str(i+1)))

            #print(np.dot(np.linalg.inv(T_0_shi), T_0_wrist1)*1000)

            T_0_wrist2 = myLeg.leg_fk_direct_calculation(0.0, [shoulder_angle,q1,q2,th4,th5,th6], i, 8, use_quaternion = False)
            T_0_wrist2[0:3,3] = T_0_wrist2[0:3,3] / 1000.0
            br.sendTransform(publish_to_tf(T_0_wrist2,"Bpysilvia", "wrist2"+str(i+1)))

            T_0_wrist3 = myLeg.leg_fk_direct_calculation(0.0, [shoulder_angle,q1,q2,th4,th5,th6], i, 9, use_quaternion = False)
            T_0_wrist3[0:3,3] = T_0_wrist3[0:3,3] / 1000.0
            br.sendTransform(publish_to_tf(T_0_wrist3,"Bpysilvia", "wrist3"+str(i+1)))

            T_0_wrist3 = myLeg.leg_fk_direct_calculation(0.0, [shoulder_angle,q1,q2,th4,th5,th6], i, 10, use_quaternion = False)
            T_0_wrist3[0:3,3] = T_0_wrist3[0:3,3] / 1000.0
            br.sendTransform(publish_to_tf(T_0_wrist3,"Bpysilvia", "GripperCenter"+str(i+1)))

            #T_0_s = myLeg.leg_fk_direct_calculation(0.0, [shoulder_angle,q1,q2,th4,th5,th6], i, 100)
            #T_0_s[0:3,3] = T_0_s[0:3,3] / 1000.0
            #br.sendTransform(publish_to_tf(T_0_s,"Bpysilvia", "shi_r"+str(i+1)))
            
            fk_res = my_scalar_k.scalar_forward_kinematics(i, [shoulder_angle,q1,q2,th4,th5,th6], with_body=True, with_gripper=True, L_actuator=L_actuator, theta_actuator=theta_actuator)
            toe1 = fk_res[0]
            toe2 = fk_res[1]
            toe1[0:3,3] = toe1[0:3,3]/1000.0
            toe2[0:3,3] = toe2[0:3,3]/1000.0
            br.sendTransform(publish_to_tf(toe1,"Bpysilvia", "toe1"+str(i+1)))
            br.sendTransform(publish_to_tf(toe2,"Bpysilvia", "toe2"+str(i+1)))


        rate.sleep()
    rospy.spin()



















