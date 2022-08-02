#! /usr/bin/env python3
import pickle
import rospy
import actionlib
import moveit_commander
import threading
import numpy as np
import quaternion as qtn
from math import pow, pi, ceil
from rosparam import load_file as ros_load_yaml
from tf_conversions import transformations as tf_trans
from sensor_msgs.msg import JointState
from moveit_msgs.msg import MoveItErrorCodes, RobotState
from geometry_msgs.msg import PoseArray, Point, Quaternion, Pose
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetStateValidity, GetStateValidityRequest
from task_msgs.srv import RoadmapSampleConfig
from task_msgs.msg import RoadmapSamplerAction, RoadmapSamplerFeedback, RoadmapSamplerResult

class RoadmapSampler:
    def __init__(self, cfg_file, tj_path, ns='', group_name='manipulator', velocity=1, planning_time=0.3, allowed_execute=False):
        self.cfg_file = cfg_file
        self.tj_path = tj_path
        self.allowed_execute = allowed_execute
        self.cfg = ros_load_yaml(cfg_file)[0][0]['sampler_config']
        self.ns = '/' + ns if ns != '' else ''
        self.group_name = group_name
        self.velocity = velocity if 0 < velocity <= 1 else 0.1
        self.planning_time = planning_time
        self._config_server = rospy.Service('set_config', RoadmapSampleConfig, self.set_config)
        # self._index_server = rospy.Service('get_pose_index', Pose, self.get_pose_index)
        self._action_server = actionlib.SimpleActionServer('sample', RoadmapSamplerAction, \
                                                           execute_cb=self.roadmap_sample_cb, auto_start=False)
        self._move_group = moveit_commander.MoveGroupCommander(name=group_name, ns = self.ns, 
                                                               robot_description = self.ns + '/robot_description')
        self._joint_state_sub = rospy.Subscriber(self.ns + '/joint_states', JointState, self._joint_states_cb)     
        self._setup(self.cfg)
        self._action_server.start()

    def _setup(self, cfg):
        self._euler_rule = self.cfg['euler_rule']
        self.sample_space = dict.fromkeys(['start', 'target'], dict())
        self.sample_space['start'] = self.construct_sample_space(cfg['start'])
        self.sample_space['target'] = self.construct_sample_space(cfg['target'])
        self.sample_space['euler_rule'] = self.cfg['euler_rule']
        self._joints_name = self._move_group.get_active_joints()
        self._init_joints = np.array(cfg['init_joints']) * np.pi / 180
        self._total_count = 0
        self._success_count = 0
        self._success_rate = 0.0
        self._sum_planning_time = 0.0
        self._sum_traj_length = 0.0
        self._analyze_data = []
        self._analyze_result = dict()

    def _ik_client(self, req):
        rospy.wait_for_service(self.ns + '/compute_ik')
        try:
            ik_client = rospy.ServiceProxy(self.ns + '/compute_ik', GetPositionIK)
            res = ik_client(req)
            return res
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def _vadility_client(self, req):
        rospy.wait_for_service(self.ns + '/check_state_validity')
        try:
            vadility_client = rospy.ServiceProxy(self.ns + '/check_state_validity', GetStateValidity)
            res = vadility_client(req)
            return res
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def _joint_states_cb(self, msg):
        self.joint_states = msg

    def save_data(self, file_path):
        data = self.sample_space
        data['trajectory'] = self.planned_trajectory
        data['analyze_result'] = self._analyze_result
        with open(file_path, 'wb') as fp:
            pickle.dump(data, fp)

    def set_config(self, req):
        s_f, s_r = self.cfg['start']['flat'], self.cfg['start']['rotation']
        t_f, t_r = self.cfg['target']['flat'], self.cfg['target']['rotation']
        d = req.start_flat.points
        s_f['points'] = [d.x, d.y, d.z]
        s_f['voxel_size'] = req.start_flat.voxel_size
        d = req.start_rotation.center
        s_r['center'] = [d.x, d.y, d.z]
        d = req.start_rotation.range
        s_r['range'] = [d.x, d.y, d.z]
        s_r['step_size'] = req.start_rotation.step_size
        d = req.target_flat.points
        t_f['points'] = [d.x, d.y, d.z]
        t_f['voxel_size'] = req.target_flat.voxel_size
        d = req.target_rotation.center
        t_r['center'] = [d.x, d.y, d.z]
        d = req.target_rotation.range
        t_r['range'] = [d.x, d.y, d.z]
        t_r['step_size'] = req.target_rotation.step_size
        self.sample_space['start'] = self.construct_sample_space(self.cfg['start'])
        self.sample_space['target'] = self.construct_sample_space(self.cfg['target'])

    def construct_sample_space(self, cfg):
        ss = dict({'pos': dict(), 'rot': dict()})
        points = np.array(cfg['flat']['points'])
        ss['pos']['origin'] = points[0]
        vec = [points[1] - points[0], points[2] - points[0]]
        # make v1 vertical to v0
        v_0_len = np.linalg.norm(vec[0])
        vec[1] = vec[1] - (vec[0] * np.dot(vec[1], vec[0]) / pow(v_0_len, 2)) if v_0_len > 0 else vec[1]
        # +1 because we need number of corners not voxels.
        ss['pos']['shape'] = [ceil(np.linalg.norm(v) / vs) + 1 if vs > 0 else 1 for v, vs in zip(vec, cfg['flat']['voxel_size'])] 
        v_s = [v / float(s - 1)  if s > 1 else 0 for s, v in zip(ss['pos']['shape'], vec)]
        ss['pos']['step'] = [(vs / np.linalg.norm(v)) * v if np.linalg.norm(v) > 0 else v for vs, v in zip(v_s, vec)]
        ss['pos']['limit'] = [np.array(ss['pos']['origin']) + (s - 1) * st for s, st in zip(ss['pos']['shape'], ss['pos']['step'])]
        ss['pos']['border_size'] = cfg['flat']['border_size']
        c_r = cfg['rotation']
        [r_c, r_r, r_s, r_b] = np.array([c_r['center'], c_r['range'], c_r['step_size'], c_r['border_size']]) * np.pi / 180
        q = tf_trans.quaternion_from_euler(r_c[0], r_c[1], r_c[2], self._euler_rule)
        r_c = tf_trans.euler_from_quaternion(q, self._euler_rule)
        # +1 for same thing, *2 for two directions of rotation.
        ss['rot']['shape'] = [ceil(r / sst) * 2 + 1 if sst > 0 else 1 for r, sst in zip(r_r, r_s)]
        ss['rot']['step'] = [r / (float(s - 1) / 2) if s - 1 > 0 else 0 for r, s in zip(r_r, ss['rot']['shape'])]
        ss['rot']['origin'] = [c - float(s - 1) / 2 * st for c, s, st in zip(r_c, ss['rot']['shape'], ss['rot']['step'])]
        ss['rot']['limit'] = [c + float(s - 1) / 2 * st for c, s, st in zip(r_c, ss['rot']['shape'], ss['rot']['step'])]
        ss['rot']['border_size'] = r_b
        print(ss)
        return ss

    # def get_pose_index(self, req):
    #     pass

    def roadmap_sample_cb(self, goal):
        self.cfg = ros_load_yaml(self.cfg_file)
        (s_rate, p_time, t_length) = self.roadmap_sample(goal.planner, goal.repeat_times, True)
        self.save_data(self.tj_path)
        self._action_server.set_succeeded(RoadmapSamplerResult(s_rate, p_time, t_length))

    def roadmap_sample(self, planner, repeat_times, feedback=False):
        self._move_group.set_planner_id(planner)
        self._move_group.set_planning_time(self.planning_time)
        self._move_group.set_max_velocity_scaling_factor(0.1)
        self._move_group.set_max_acceleration_scaling_factor(0.1)
        start_poses_num = self.compute_sample_space_elements(self.sample_space['start'])
        target_poses_num = self.compute_sample_space_elements(self.sample_space['target'])
        total_poses_num = start_poses_num * target_poses_num
        self.planned_trajectory = np.full([start_poses_num, target_poses_num], None)
        index_list = np.array(range(total_poses_num))
        s_rate, p_time, t_length = 0., 0., 0.
        ik_failed, error_12, state_invalid = 0, 0, 0
        invalid_list = []
        for ep in range(repeat_times):
            index_list = np.delete(index_list, invalid_list)
            np.random.shuffle(index_list)
            total_poses_num = len(index_list)
            invalid_list = []
            for i, index in enumerate(index_list):
                i_s, i_t = index // target_poses_num, index % target_poses_num
                idxs_s = self.get_indexes_from_index(i_s, self.sample_space['start'])
                idxs_t = self.get_indexes_from_index(i_t, self.sample_space['target'])
                pos_s, rot_s = self.get_pose_from_indexes(idxs_s, self.sample_space['start'])
                pos_t, rot_t = self.get_pose_from_indexes(idxs_t, self.sample_space['target'])
                ik_result = self.compute_ik(pos_s, rot_s, self._init_joints)
                if ik_result.error_code.val != MoveItErrorCodes.SUCCESS:
                    ik_failed+=1
                    # long info
                    # print('[Roadmap Sampler]: {}/{}, start pose IK error: {}, num of ik failed = {},\n{} pos_s = {}, rot_s = {}' \
                    #     .format(i+ep*total_poses_num + 1, total_poses_num * repeat_times, ik_result.error_code, ik_failed, ' ' * 18, \
                    #             pos_s, np.around(np.array(rot_s) * 180 / pi, 1)))
                    print('[Roadmap Sampler]: {}/{}, start pose IK error: {}, num of ik failed = {}' \
                        .format(i+ep*total_poses_num + 1, total_poses_num * repeat_times, ik_result.error_code, ik_failed))
                    # invalid_list.append(i)
                    continue

                start_joints = ik_result.solution.joint_state.position
                validity = self.check_validity(start_joints)
                if not validity.valid:
                    state_invalid+=1
                    # long info
                    # print('[Roadmap Sampler]: {}/{}, start pose invalid, num of invalid state = {},\n{} pos_s = {}, rot_s = {}' \
                    #     .format(i+ep*total_poses_num + 1, total_poses_num * repeat_times, state_invalid, ' ' * 18, \
                    #             pos_s, np.around(np.array(rot_s) * 180 / pi, 1)))
                    print('[Roadmap Sampler]: {}/{}, start pose invalid, num of invalid state = {}' \
                        .format(i+ep*total_poses_num + 1, total_poses_num * repeat_times, state_invalid))
                    invalid_list.append(i)
                    continue

                ik_result = self.compute_ik(pos_t, rot_t, start_joints)
                if ik_result.error_code.val != 1:
                    ik_failed+=1
                    # long info
                    # print('[Roadmap Sampler]: {}/{}, target pose IK error: {}, num of ik failed = {},\n{} pos_t = {}, rot_t = {}' \
                    #     .format(i+ep*total_poses_num + 1, total_poses_num * repeat_times, ik_result.error_code, ik_failed, ' ' * 18, \
                    #             pos_t, np.around(np.array(rot_t) * 180 / pi, 1)))
                    print('[Roadmap Sampler]: {}/{}, target pose IK error: {}, num of ik failed = {}' \
                        .format(i+ep*total_poses_num + 1, total_poses_num * repeat_times, ik_result.error_code, ik_failed))
                    # invalid_list.append(i)
                    continue

                target_joints = ik_result.solution.joint_state.position
                validity = self.check_validity(target_joints)
                if not validity.valid:
                    state_invalid+=1
                    # long time
                    # print('[Roadmap Sampler]: {}/{}, target pose invalid, num of invalid state = {},\n{} pos_s = {}, rot_s = {}' \
                    #     .format(i+ep*total_poses_num + 1, total_poses_num * repeat_times, state_invalid, ' ' * 18, \
                    #             pos_s, np.around(np.array(rot_s) * 180 / pi, 1)))
                    print('[Roadmap Sampler]: {}/{}, target pose invalid, num of invalid state = {}' \
                        .format(i+ep*total_poses_num + 1, total_poses_num * repeat_times, state_invalid))
                    invalid_list.append(i)
                    continue

                result = self.moveit_plan(start_joints, target_joints)
                (s_rate, p_time, t_length) = self.analyze_result(result)

                if result[3].val == MoveItErrorCodes.SUCCESS:
                    print('[Roadmap Sampler]: {}/{}, planning successed, success rate: {}, planning time: {}, traj length: {}' \
                        .format(i+ep*total_poses_num + 1, total_poses_num * repeat_times, s_rate, p_time, t_length))
                    self.planned_trajectory[i_s, i_t] = result[1].joint_trajectory.points
                else:
                    if result[3].val == -12:
                        error_12 += 1
                    # long info
                    # print('[Roadmap Sampler]: {}/{}, planning failed, error_code: {}, num of invalid goal = {},\n{} pos_s = {}, rot_s = {}, pos_t = {}, rot_t = {}' \
                    #     .format(i+ep*total_poses_num + 1, total_poses_num * repeat_times, result[3].val, error_12, ' ' * 18, \
                    #             pos_s, np.around(np.array(rot_s) * 180 / pi, 1), pos_t, np.around(np.array(rot_t) * 180 / pi, 1)))
                    print('[Roadmap Sampler]: {}/{}, planning failed, error_code: {}, num of invalid goal = {}' \
                        .format(i+ep*total_poses_num + 1, total_poses_num * repeat_times, result[3].val, error_12))
                if feedback:
                    self.action_feedback(i+ep*total_poses_num + 1, total_poses_num * repeat_times)
            self.reset_analyze_count()
        self._analyze_result['success_rate'] = s_rate
        self._analyze_result['planning_time'] = p_time
        self._analyze_result['trajectory_length'] = t_length
        return (s_rate, p_time, t_length)

    def continuous_sample(self, planner, repeat_times, feedback=False):
        self._move_group.set_planner_id(planner)
        self._move_group.set_planning_time(self.planning_time)
        start_poses_num = self.compute_sample_space_elements(self.sample_space['start'])
        target_poses_num = self.compute_sample_space_elements(self.sample_space['target'])
        total_poses_num = start_poses_num * target_poses_num
        index_list = np.array(range(total_poses_num))
        s_rate, p_time, t_length = 0., 0., 0.
        ik_failed, error_12, state_invalid = 0, 0, 0
        invalid_list = []
        go_to_start = False
        self.replan = False
        init_joints = self.check_joint_state(self.joint_states)
        goal_joints_tmp = []
        execute_thread = threading.Thread(target = self.moveit_execute, args = (None,))
        rate = rospy.Rate(2)
        for ep in range(repeat_times):
            index_list = np.delete(index_list, invalid_list)
            np.random.shuffle(index_list)
            total_poses_num = len(index_list)
            invalid_list = []
            for i, index in enumerate(index_list):
                if go_to_start: # and not self.replan:
                    i_s = index // target_poses_num
                    idxs_s = self.get_indexes_from_index(i_s, self.sample_space['start'])
                    pos_s, rot_s = self.get_pose_from_indexes(idxs_s, self.sample_space['start'])
                    ik_result = self.compute_ik(pos_s, rot_s, init_joints)
                    if ik_result.error_code.val != MoveItErrorCodes.SUCCESS:
                        ik_failed+=1
                        print('[Roadmap Sampler]: {}/{}, start pose IK error: {}, num of ik failed = {}' \
                            .format(i+ep*total_poses_num + 1, total_poses_num * repeat_times, ik_result.error_code, ik_failed))
                        invalid_list.append(i)
                        continue

                    goal_joints = ik_result.solution.joint_state.position
                    validity = self.check_validity(goal_joints)
                    if not validity.valid:
                        state_invalid+=1
                        print('[Roadmap Sampler]: {}/{}, start pose invalid, num of invalid state = {}' \
                            .format(i+ep*total_poses_num + 1, total_poses_num * repeat_times, state_invalid))
                        invalid_list.append(i)
                        continue
                # elif not self.replan:
                else:
                    i_t = index % target_poses_num
                    idxs_t = self.get_indexes_from_index(i_t, self.sample_space['target'])
                    pos_t, rot_t = self.get_pose_from_indexes(idxs_t, self.sample_space['target'])
                    ik_result = self.compute_ik(pos_t, rot_t, init_joints)
                    if ik_result.error_code.val != 1:
                        ik_failed+=1
                        print('[Roadmap Sampler]: {}/{}, target pose IK error: {}, num of ik failed = {}' \
                            .format(i+ep*total_poses_num + 1, total_poses_num * repeat_times, ik_result.error_code, ik_failed))
                        invalid_list.append(i)
                        continue

                    goal_joints = ik_result.solution.joint_state.position
                    validity = self.check_validity(goal_joints)
                    if not validity.valid:
                        state_invalid+=1
                        print('[Roadmap Sampler]: {}/{}, target pose invalid, num of invalid state = {}' \
                            .format(i+ep*total_poses_num + 1, total_poses_num * repeat_times, state_invalid))
                        invalid_list.append(i)
                        continue
                if execute_thread.isAlive():
                    execute_thread.join()
                if self.replan:
                    goal_joints = goal_joints_tmp

                if self.allowed_execute:
                    init_joints = self.check_joint_state(self.joint_states)
                result = self.moveit_plan(init_joints, goal_joints)
                (s_rate, p_time, t_length) = self.analyze_result(result)

                if result[3].val == MoveItErrorCodes.SUCCESS:
                    print('[Roadmap Sampler]: {}/{}, planning successed, success rate: {}, planning time: {}, traj length: {}' \
                        .format(i+ep*total_poses_num + 1, total_poses_num * repeat_times, s_rate, p_time, t_length))
                    if self.replan is False:
                        go_to_start = not go_to_start
                    init_joints = goal_joints
                    goal_joints_tmp = [x for x in goal_joints]
                    if self.allowed_execute:
                        self.replan = False
                        self.moveit_execute(result[1])
                        # execute_thread = threading.Thread(target = self.moveit_execute, args = (result[1],))
                        # execute_thread.start()
                else:
                    if result[3].val == -12:
                        error_12 += 1
                    print('[Roadmap Sampler]: {}/{}, planning failed, error_code: {}, num of invalid goal = {}' \
                        .format(i+ep*total_poses_num + 1, total_poses_num * repeat_times, result[3].val, error_12))
                if feedback:
                    self.action_feedback(i+ep*total_poses_num + 1, total_poses_num * repeat_times)
                rate.sleep()
        return

    def check_joint_state(self, js):
        joints_position = []
        for jn in self._joints_name:
            idx = js.name.index(jn)
            joints_position.append(js.position[idx])
        return joints_position

    def action_feedback(self, finished, total):
        feedback = RoadmapSamplerFeedback(finished, total)
        self._action_server.publish_feedback(feedback)

    def compute_sample_space_elements(self, sample_space):
        dim = sample_space['pos']['shape'] + sample_space['rot']['shape']
        return np.prod(dim)

    def get_pose_from_indexes(self, indexes, sample_space):
        sp, sr = sample_space['pos'], sample_space['rot']
        
        pos = [x for x in sp['origin']]
        for i, s in enumerate(sp['step']):
            pos += indexes[i] * s        
        rot = np.array([ro + idx * rs for ro, idx, rs in zip(sr['origin'], indexes[len(sp['step']):], sr['step'])])

        return pos, rot

    def get_indexes_from_index(self, index, sample_space):
        indexes = []
        shape = sample_space['pos']['shape'] + sample_space['rot']['shape']
        for i in range(len(shape)):
            dim_size = np.prod(shape[i+1 : ]) if i+1 < len(shape) else 1
            a, b = index // dim_size, index % dim_size
            indexes.append(a)
            index = b
        return indexes

    def pop_shuffled_index(self, index_list, sample_space):
        np.random.shuffle(index_list)
        index = index_list[0]
        index_list = index_list[1:]
        shape = sample_space['pos']['shape'] + sample_space['rot']['shape']
        dim_size, indexes = [], []
        for i in range(len(shape)):
            dim_size.append(np.prod(shape[i+1 : ]) if i+1 < len(shape) else 1)
            dim_size = np.prod(shape[i+1 : ]) if i+1 < len(shape) else 1
            a, b = index // dim_size, index % dim_size
            indexes.append(a)
            index = b
        return indexes

    def check_validity(self, joints):
        req = GetStateValidityRequest()
        req.group_name = self.group_name
        req.robot_state = self.generate_robot_state(self._joints_name, joints)
        result = self._vadility_client(req)
        return result

    def compute_ik(self, pos, rot, init_joints):
        # q0 = qtn.from_euler_angles(r)
        q = tf_trans.quaternion_from_euler(rot[0], rot[1], rot[2], self._euler_rule)
        req = GetPositionIKRequest()
        req.ik_request.group_name = self.group_name
        req.ik_request.ik_link_name = '/tool_tip'
        req.ik_request.avoid_collisions = False
        req.ik_request.robot_state = self.generate_robot_state(self._joints_name, init_joints)
        req.ik_request.pose_stamped.header.stamp = rospy.Time.now()
        req.ik_request.pose_stamped.pose.position = Point(pos[0], pos[1], pos[2])
        # req.ik_request.pose_stamped.pose.orientation = Quaternion(q.x, q.y, q.z, q.w)
        req.ik_request.pose_stamped.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
        result = self._ik_client(req)
        return result

    def moveit_plan(self, start_joints, target_joints):
        sj, tj = np.array(start_joints), np.array(target_joints)
        if np.any(np.absolute(sj) > np.pi * 2):
            sj = sj * np.pi / 180
        if np.any(np.absolute(tj) > np.pi * 2):
            tj = tj * np.pi / 180
        self._move_group.set_max_velocity_scaling_factor(self.velocity)
        self._move_group.set_max_acceleration_scaling_factor(self.velocity)
        if len(sj) == len(self._joints_name):
            start_state = self.generate_robot_state(self._joints_name, sj)
            self._move_group.set_start_state(start_state)

        self._move_group.set_joint_value_target(tj)
        return self._move_group.plan()

    def moveit_execute(self, traj):
        success = self._move_group.execute(traj)
        if success:
            self.replan = False
        else:
            self.replan = True

    def analyze_result(self, result):
        error_code = result[3].val
        if error_code == MoveItErrorCodes.SUCCESS or \
            error_code == MoveItErrorCodes.INVALID_MOTION_PLAN or \
            error_code == MoveItErrorCodes.PLANNING_FAILED or \
            error_code == MoveItErrorCodes.TIMED_OUT:
            self._total_count += 1

        if error_code == MoveItErrorCodes.SUCCESS and len(result[1].joint_trajectory.points) > 1:
            self._success_count += 1
            
            self._sum_planning_time += result[2]
            
            jtps = result[1].joint_trajectory.points
            traj_length = 0.0
            base_traj_length = np.linalg.norm(np.array(jtps[-1].positions) - np.array(jtps[0].positions))
            for i, jtp in enumerate(jtps):
                if jtp == jtps[-1]:
                    break
                traj_length += np.linalg.norm(np.array(jtp.positions) - np.array(jtps[i+1].positions))
            
            self._sum_traj_length += (traj_length / base_traj_length)
        success_rate = self._success_count / self._total_count if self._total_count > 0 else 0
        avg_planning_time = self._sum_planning_time / self._success_count if self._success_count > 0 else 0
        avg_traj_length = self._sum_traj_length / self._success_count if self._success_count > 0 else 0

        return round(success_rate,4), round(avg_planning_time, 4), round(avg_traj_length, 4)

    def reset_analyze_count(self):
        ad, t, s, p, l = self._analyze_data, self._total_count, self._success_count, self._sum_planning_time, self._sum_traj_length
        ad.append([round(s / t, 4), round(p / s, 4), round(l / s, 4)])
        self._total_count, self._success_count, self._sum_planning_time, self._sum_traj_length = 0.0, 0.0, 0.0, 0.0
        print('======================================================================================')
        for i, da in enumerate(ad):
            print('[Roadmap Sampler]: Round {} result: success rate: {}, planning time: {}, traj length: {}'.format(i+1, da[0], da[1], da[2]))
        print('======================================================================================')

    def generate_robot_state(self, joints_name, start_joints):
        if type(start_joints) == JointState:
            joint_state = start_joints
        else:
            joint_state = JointState()
            joint_state.header.stamp = rospy.Time.now()
            joint_state.name = joints_name
            joint_state.position = start_joints
        state = RobotState()
        state.joint_state = joint_state
        return state