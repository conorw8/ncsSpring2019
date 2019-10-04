#!/bin/bash
export ROS_MASTER_URI=http://$(cat ~/master.cfg):11311
export ROS_IP=$(~/getmyip.sh)
export ROS_HOSTNAME=$ROS_IP #`hostname`
#export ROS_NAMESPACE=/`echo $ROS_HOSTNAME | sed -e 's/-/_/g'`;

