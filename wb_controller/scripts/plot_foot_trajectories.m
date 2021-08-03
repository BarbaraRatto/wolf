clear all
close all

load rt_logger_TrajectoryInterface_1_position_reference_1_position_reference
load rt_logger_TrajectoryInterface_2_position_reference_2_position_reference
load rt_logger_TrajectoryInterface_3_position_reference_3_position_reference
load rt_logger_TrajectoryInterface_4_position_reference_4_position_reference

load rt_logger_TrajectoryInterface_1_position_reference_time
load rt_logger_TrajectoryInterface_2_position_reference_time
load rt_logger_TrajectoryInterface_3_position_reference_time
load rt_logger_TrajectoryInterface_4_position_reference_time


foot_1 = rt_logger_TrajectoryInterface_1_position_reference_1_position_reference;
foot_2 = rt_logger_TrajectoryInterface_2_position_reference_2_position_reference;
foot_3 = rt_logger_TrajectoryInterface_3_position_reference_3_position_reference;
foot_4 = rt_logger_TrajectoryInterface_4_position_reference_4_position_reference;

t1 = rt_logger_TrajectoryInterface_1_position_reference_time;
t2 = rt_logger_TrajectoryInterface_2_position_reference_time;
t3 = rt_logger_TrajectoryInterface_3_position_reference_time;
t4 = rt_logger_TrajectoryInterface_4_position_reference_time;

figure(1)
hold on
plot3(foot_1(:,1),foot_1(:,2),foot_1(:,3),'r')
plot3(foot_2(:,1),foot_2(:,2),foot_2(:,3),'b')
plot3(foot_3(:,1),foot_3(:,2),foot_3(:,3),'y')
plot3(foot_4(:,1),foot_4(:,2),foot_4(:,3),'g')
axis equal
hold off