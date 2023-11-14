syms joint1 joint2 joint3 wrist1 joint1_joint2_z_offset joint2_joint3_x_offset joint3_wrist1_x_offset wrist1_cam_x_offset

T_joint1 = sym(eye(4));
T_joint1(1:3,1:3) = rotz(joint1);
T_joint1_joint2 = sym(eye(4));
T_joint1_joint2(1:3,1:3) =rotz(pi/2)* roty(-pi/2);
T_joint1_joint2(3,4) = joint1_joint2_z_offset;

T_joint2 = sym(eye(4));
T_joint2(1:3,1:3) = rotz(joint2);

T_joint2_joint3 = sym(eye(4));
T_joint2_joint3(1,4) = joint2_joint3_x_offset;

T_joint3 = sym(eye(4));
T_joint3(1:3,1:3) = rotz(joint3);

T_joint3_wrist1 = sym(eye(4));
T_joint3_wrist1(1,4) = joint3_wrist1_x_offset;

T_wrist1 = sym(eye(4));
T_wrist1(1:3,1:3) = rotz(wrist1);

T_final = [0, 0, 1,wrist1_cam_x_offset; 
           0,-1, 0,0; 
           1, 0, 0,0; 
           0, 0, 0,1];

T = T_joint1*T_joint1_joint2*T_joint2*T_joint2_joint3*T_joint3*T_joint3_wrist1*T_wrist1*T_final;
T = simplify(T);

T