#!/bin/sh

# determine if we're in the devel or install space
if [ "False" = "True" -o "False" = "true" ]
then
  PLUGINS=/home/rocky/catkin_ws/devel/lib/openrave-0.9
else
  PLUGINS=/home/rocky/catkin_ws/install/lib/openrave-0.9
fi

# prepend to paths (if not already there)
# from http://unix.stackexchange.com/a/124447
case ":${OPENRAVE_PLUGINS:=$PLUGINS}:" in
    *:$PLUGINS:*) ;;
    *) OPENRAVE_PLUGINS="$PLUGINS:$OPENRAVE_PLUGINS" ;;
esac

export OPENRAVE_PLUGINS
