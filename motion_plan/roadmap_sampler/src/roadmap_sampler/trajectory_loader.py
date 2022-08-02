#! /usr/bin/env python3
import pickle
import rospy
import numpy as np
import quaternion as qtn
from os.path import exists
from tf_conversions import transformations as tf_trans
from task_msgs.srv import GetPlannedTraj, GetPlannedTrajResponse
# for debug ================================
# import moveit_commander
# from moveit_msgs.srv import GetPositionFK
# from std_msgs.msg import Header
# from moveit_msgs.msg import MoveItErrorCodes, RobotState
# from sensor_msgs.msg import JointState
# ==========================================

class TrajectoryLoader:
    def __init__(self, file_path):
        self._traj_getter_server = rospy.Service('get_planned_trajectory', GetPlannedTraj, self.get_planned_trajectory)
        self._load_data(file_path)
    
    def _load_data(self, file_path):
        self.sample_space = dict()
        if exists(file_path):
            with open(file_path,'rb') as fp:
                data = pickle.load(fp) 
            self.sample_space['start'] = data['start']
            self.sample_space['target'] = data['target']
            self.euler_rule = data['euler_rule']
            self.planned_trajectory = data['trajectory']
        else:
            self.sample_space = None
            rospy.logerr('[TrajectoryLoader]: Data file does not exist')
        # for debug =======================================
        # self.fk_srv = rospy.ServiceProxy('/robot0/compute_fk', GetPositionFK)
        # self._move_group = moveit_commander.MoveGroupCommander(name='manipulator', ns = '/robot0', 
        #                                                        robot_description = '/robot0/robot_description')
        # self._joints_name = self._move_group.get_active_joints()
        # =================================================

    def get_planned_trajectory(self, req):
        res = GetPlannedTrajResponse()
        res.success = False
        if self.sample_space is None:
            rospy.logerr('[TrajectoryLoader]: Sample space does not exist')
            return res

        start_indexes = self.get_indexes_from_pose(req.start.position, req.start.orientation, self.sample_space['start'])
        target_indexes = self.get_indexes_from_pose(req.target.position, req.target.orientation, self.sample_space['target'])
        if start_indexes and target_indexes:
            start_index = self.get_index_from_indexes(start_indexes, self.sample_space['start'])
            target_index = self.get_index_from_indexes(target_indexes, self.sample_space['target'])
            points = self.planned_trajectory[start_index, target_index]
            if points is not None:
                res.success = True
                res.traj_points = points
                # for debug ======================================
                # end_pose = self.call_compute_fk(points[-1].positions, ['robot0/tcp0'])
                # print('end_pose = {}, points[-1].positions = {}'.format(end_pose, points[-1].positions))
                # ================================================
            else:
                rospy.logwarn('[TrajectoryLoader]: points is None')
        return res

    def get_indexes_from_pose(self, pos, quat, sample_space):
        sp, sr = sample_space['pos'], sample_space['rot']
        p = [pos.x, pos.y, pos.z]
        r = tf_trans.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w], self.euler_rule)
        r = self.check_rot_quadrant(r, sr['origin'], sr['limit'])
        result = []
        print('v = {}, o = {}, s = {}'.format(list(p) + list(r), list(sp['origin']) + sr['origin'], sp['step'] + sr['step']))
        for i, (s, sh) in enumerate(zip(sp['step'], sp['shape'])):
            o, s_l = np.array(sp['origin']), np.linalg.norm(s)
            idx = round((np.dot(p - o, s) / (s_l * s_l))) if abs(s_l) > 0.00001 else 0
            if idx < 0 or idx > sh - 1:
                if self.check_pos_limit(idx * s_l, sp['origin'], sp['limit'][i], sp['border_size'][i]):
                    idx = 0 if idx < 0 else sh - 1
                else:
                    rospy.logerr('[TrajectoryLoader]: Position requesst out of the range of trajectory map')
                    return None
            result.append(idx)

        for i, (v, o, s, sh) in enumerate(zip(r, sr['origin'], sr['step'], sr['shape'])):
            idx = round(abs((v - o) / s)) if abs(s) > 0.00001 else 0
            if idx < 0 or idx > sh - 1:
                if self.check_rot_limit(v, o, sr['limit'][i], sr['border_size'][i]):
                    idx = 0 if idx < 0 else sh - 1
                else:
                    rospy.logerr('[TrajectoryLoader]: Rotation requesst out of the range of trajectory map')
                    return None
            result.append(idx)
        return result

    def get_index_from_indexes(self, indexes, sample_space):
        shape = sample_space['pos']['shape'] + sample_space['rot']['shape']
        index = 0.
        if len(shape) == len(indexes):
            for i in range(len(shape) -1):
                index += indexes[i] * np.prod(shape[i+1:])
            index += indexes[-1]
            return int(index)
        else:
            rospy.logerr('[TrajectoryLoader]: Lengths of shape and indexes art not equal')
            return None

    def check_pos_limit(self, proj_length, origin, limit, border_size):
        if proj_length < 0 and abs(proj_length) > border_size:
            return False
        elif proj_length > np.linalg.norm(limit - origin) + border_size:
            return False
        else:
            return True

    def check_rot_limit(self, rot, origin, limit, border_size):
        if rot < origin - border_size or rot > limit + border_size:
            return False
        else:
            return True
        

    def check_rot_quadrant(self, rot, origin, limit):
        center = (np.array(origin) + np.array(limit)) / 2
        result = []
        for r, c in zip(rot, center):
            if r - c > np.pi:
                result.append(r - 2 * np.pi)
            elif r - c < -1 * np.pi:
                result.append(r + 2 * np.pi)
            else:
                result.append(r)
        return result

    # for debug =====================================================
    # def call_compute_fk(self, joint_pose, links, joint_names=None):
    #     try:
    #         header = Header()
    #         header.stamp = rospy.Time.now()
    #         header.frame_id = 'world'

    #         rs = self.generate_robot_state(self._joints_name, joint_pose)

    #         res = self.fk_srv(header, links, rs)
    #         pose_stamped = None
    #         if res.error_code.val == MoveItErrorCodes.SUCCESS:
    #             pose_stamped = res.pose_stamped
    #         else:
    #             rospy.logerr('[FK Service] solve fk failed. error code: {}'.format(res.error_code))
    #         # list of pose_stamped 
    #         return pose_stamped

    #     except (Exception, rospy.ServiceException) as e:
    #         rospy.logerr('[FK Service] call failed: {}'.format(e))

    # def generate_robot_state(self, joints_name, start_joints):
    #     if type(start_joints) == JointState:
    #         joint_state = start_joints
    #     else:
    #         joint_state = JointState()
    #         joint_state.header.stamp = rospy.Time.now()
    #         joint_state.name = joints_name
    #         joint_state.position = start_joints
    #     state = RobotState()
    #     state.joint_state = joint_state
    #     print('state = {}'.format(state))
    #     return state
    # ==============================================================