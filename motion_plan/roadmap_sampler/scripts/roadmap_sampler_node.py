#! /usr/bin/env python3
import rospy
from roadmap_sampler import RoadmapSampler, TrajectoryLoader

PLANNER = 'AdaptPRM'
REPEAT = 10

if __name__ == '__main__':
    rospy.init_node('roadmap_sampler')
    # cfg = rospy.get_param('sampler_config')
    robot_name = rospy.get_param('~robot_name')
    cfg_file = rospy.get_param('~cfg_file')
    run = rospy.get_param('~run')
    tj_storage = rospy.get_param('~tj_storage')
    tj_load = rospy.get_param('~tj_load')
    allowed_execute = rospy.get_param('~allowed_execute')
    continuous_mode = rospy.get_param('~continuous_mode')
    roadmap_sampler = RoadmapSampler(cfg_file, tj_storage, ns=robot_name, allowed_execute=allowed_execute)
    trajectory_loader = TrajectoryLoader(tj_load)
    if run:
        if continuous_mode:
            roadmap_sampler.continuous_sample(PLANNER, REPEAT)
        else:
            roadmap_sampler.roadmap_sample(PLANNER, REPEAT)
        roadmap_sampler.save_data(tj_storage)
    else:
        rospy.spin()
