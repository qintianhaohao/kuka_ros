#!/usr/bin/env python
# -*- coding: utf-8 -*-

# 笛卡尔运动规划

import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose
from copy import deepcopy

class MoveItCartesianDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化ROS节点
        rospy.init_node('moveit_cartesian_demo', anonymous=True)
        
        # 是否需要使用笛卡尔空间的运动规划
        cartesian = rospy.get_param('~cartesian', True)
                        
        # 初始化需要使用move group控制的机械臂中的arm group
        arm = MoveGroupCommander('kr10')
        
        # 当运动规划失败后，允许重新规划
        arm.allow_replanning(True)
        
        # 设置目标位置所使用的参考坐标系
        arm.set_pose_reference_frame('base_link')
                
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        arm.set_goal_position_tolerance(0.01)
        arm.set_goal_orientation_tolerance(0.1)
        
        # 获取终端link的名称
        end_effector_link = arm.get_end_effector_link()
                                        
        # 控制机械臂运动到之前设置的“forward”姿态
        arm.set_named_target('home')
        arm.go()
        
        # 获取当前位姿数据最为机械臂运动的起始位姿
        start_pose = arm.get_current_pose(end_effector_link).pose

        # 路径
        waypoints = self.get_waypoints(start_pose)

        if cartesian:
            fraction = 0.0   #路径规划覆盖率
            maxtries = 100   #最大尝试规划次数
            attempts = 0     #已经尝试规划次数
            
            # 设置机器臂当前的状态作为运动初始状态
            arm.set_start_state_to_current_state()
     
            # 尝试规划一条笛卡尔空间下的路径，依次通过所有路点
            while fraction < 1.0 and attempts < maxtries:
                (plan, fraction) = arm.compute_cartesian_path (
                                        waypoints,   # waypoint poses，路点列表
                                        0.01,        # eef_step，终端步进值
                                        0.0,         # jump_threshold，跳跃阈值
                                        True)        # avoid_collisions，避障规划
                
                # 尝试次数累加
                attempts += 1
                
                # 打印运动规划进程
                if attempts % 10 == 0:
                    rospy.loginfo("Still trying after " + str(attempts) + " attempts...")
                         
            # 如果路径规划成功（覆盖率100%）,则开始控制机械臂运动
            if fraction == 1.0:
                rospy.loginfo("Path computed successfully. Moving the arm.")
                arm.execute(plan)
                rospy.loginfo("Path execution complete.")
            # 如果路径规划失败，则打印失败信息
            else:
                rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")  

        # 控制机械臂回到初始化位置
        arm.set_named_target('home')
        arm.go()
        rospy.sleep(1)
        
        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

    def get_waypoints(self, start_pose):
                
        # 初始化路点列表
        waypoints = []
                
        # 将初始位姿加入路点列表
        waypoints.append(start_pose)
            
        wpose = deepcopy(start_pose)
        wpose.position.y += 0.05
        wpose.position.z -= 0.014
        waypoints.append(deepcopy(wpose))
         
        wpose.position.y += 0.036
        wpose.position.z -= 0.036
        waypoints.append(deepcopy(wpose))
        
        wpose.position.y += 0.014
        wpose.position.z -= 0.05
        waypoints.append(deepcopy(wpose))

        wpose.position.y -= 0.014
        wpose.position.z -= 0.05
        waypoints.append(deepcopy(wpose))

        wpose.position.y -= 0.036
        wpose.position.z -= 0.036
        waypoints.append(deepcopy(wpose))

        wpose.position.y -= 0.05
        wpose.position.z -= 0.014
        waypoints.append(deepcopy(wpose))

        wpose.position.y -= 0.05
        wpose.position.z += 0.014
        waypoints.append(deepcopy(wpose))

        wpose.position.y -= 0.036
        wpose.position.z += 0.036
        waypoints.append(deepcopy(wpose))

        wpose.position.y -= 0.014
        wpose.position.z += 0.05
        waypoints.append(deepcopy(wpose))

        wpose.position.y += 0.014
        wpose.position.z += 0.05
        waypoints.append(deepcopy(wpose))

        wpose.position.y += 0.036
        wpose.position.z += 0.036
        waypoints.append(deepcopy(wpose))

        wpose.position.y += 0.05
        wpose.position.z += 0.014
        waypoints.append(deepcopy(wpose))

        
        # 回到初始位置，并加入路点列表
        waypoints.append(deepcopy(start_pose))

        return waypoints
            



if __name__ == "__main__":
    try:
        MoveItCartesianDemo()
    except rospy.ROSInterruptException:
        pass
