#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
ROS node that publishes the transform to the pozyx tag
"""

import pypozyx
import rospy
from std_msgs.msg import Header
#from geometry_msgs.msg import Point, Pose, Quaternion
import tf
remote_id = None

class ROSPozyx:
    def __init__(self,pozyx, anchors, algorithm=pypozyx.POZYX_POS_ALG_UWB_ONLY, dimension=pypozyx.POZYX_3D, height=1000,remote_id=None):
        self.pozyx = pozyx
        self.anchors = anchors
        self.remote_id = remote_id
        self.seq=0
        self.setAnchorsManual()
        self.tb = tf.TransformBroadcaster()
        self.pos = None
        self.quat = None


    def setAnchorsManual(self):
        """Adds the manually measured anchors to the Pozyx's device list one for one."""
        #status = self.pozyx.clearDevices(self.remote_id)
        for anchor in self.anchors:
            self.pozyx.addDevice(anchor, self.remote_id)
        if len(self.anchors) > 4:
            self.pozyx.setSelectionOfAnchors(pypozyx.POZYX_ANCHOR_SEL_AUTO, len(self.anchors))
        #return status
    
    def smoothData(self,pos,quat):
        if self.pos is None:
            self.pos = pos
        if self.quat is None:
            self.quat = quat
        self.pos = [p1*0.9+p2*0.1 for p1,p2 in zip(self.pos,pos)]
        self.quat = [p1*0.9+p2*0.1 for p1,p2 in zip(self.quat,quat)]
            
    
    def pozyx_pose_pub(self):
        while not rospy.is_shutdown():
            coords = pypozyx.Coordinates()
            quat = pypozyx.Quaternion()
            self.pozyx.doPositioning(coords, pypozyx.POZYX_3D, remote_id=self.remote_id)
            self.pozyx.getQuaternion(quat, remote_id=self.remote_id)
            pos = [coords.x/1000.0,coords.y/1000.0,coords.z/1000.0]
            quat = [quat.x, quat.y, quat.z, quat.w]
            self.smoothData(pos,quat)
            self.tb.sendTransform(self.pos,
                                  self.quat,
                                  rospy.Time.now(),
                                  "pozyx",
                                  "world"
                                  )


if __name__ == '__main__':
    rospy.init_node('pozyx_pose_node')
    port = '/dev/ttyACM0'
    #port = pypozyx.get_serial_ports()[0].device

    # necessary data for calibration, change the IDs and coordinates yourself
    anchors = [pypozyx.DeviceCoordinates(0x6e49, 1, pypozyx.Coordinates(0, 0, 835)),
               pypozyx.DeviceCoordinates(0x6e57, 1, pypozyx.Coordinates(3850, 380, 700)),
               pypozyx.DeviceCoordinates(0x6e32, 1, pypozyx.Coordinates(1375, 2150, 1110)),
               pypozyx.DeviceCoordinates(0x6e0f, 1, pypozyx.Coordinates(5350, 1500, 910))]

    algorithm = pypozyx.POZYX_POS_ALG_UWB_ONLY  # positioning algorithm to use
    dimension = pypozyx.POZYX_3D               # positioning dimension
    height = 1000                      # height of device, required in 2.5D positioning
    pozyx = pypozyx.PozyxSerial(port)
    remote_id = None #"0x6e3a"
    r = ROSPozyx(pozyx,  anchors, algorithm, dimension, height,remote_id)
    try:
        r.pozyx_pose_pub()
    except rospy.ROSInterruptException:
        pass
