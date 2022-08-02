#!/usr/bin/env python
#!\file
#
# \author  Wanqing Xia wxia612@aucklanduni.ac.nz
# \date    2022-07-28
#
#
# ---------------------------------------------------------------------

import sys
import rospy

from pycontrol.conveyor import ConveyorBelt

if __name__ == "__main__":
    rospy.init_node("combined_control")
    mm = ConveyorBelt()