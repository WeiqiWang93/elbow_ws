#!/bin/sh

for share_path in $(catkin_find --share | tac); do
    OPENRAVE_PLUGINS="${share_path}/openrave-0.9/plugins:${OPENRAVE_PLUGINS}"
    OPENRAVE_DATA="${share_path}/openrave-0.9/data:${OPENRAVE_DATA}"
    OPENRAVE_DATABASE="${share_path}/openrave-0.9/databases:${OPENRAVE_DATABASES}"
done

export OPENRAVE_PLUGINS OPENRAVE_DATA OPENRAVE_DATABASES
