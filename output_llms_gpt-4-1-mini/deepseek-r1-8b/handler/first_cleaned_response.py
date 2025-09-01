import chrono as c
   import numpy as np
   import rospy
   from std_msgs import Int32

g = 9.81  
   system = c.System(gx=0, gy=0, gz=-g)

floor_body = c.Body(name="floor", position=(0, 0, 0))
   floor_size = c.Vec3(5, 5, 1)  
   c.BodyInitSize(floor_body, floor_size)
   c.BodySetMaterial(floor_body, c.Material(young=2e11, poisson=0.3))
   system.AddBody(floor_body)

box_body = c.Body(name="box", mass=1, position=(2, 0, 1))
   box_size = c.Vec3(1, 1, 1)  
   c.BodyInitSize(box_body, box_size)
   c.BodySetMaterial(box_body, c.Material(young=2e11, poisson=0.3))
   system.AddBody(box_body)

c.BodyConnect(box_body, floor_body, connect_type=c.ConnectType.WHEEL)

class ObjectPositionHandler(c.ROSHandler):
       def __init__(self):
           super().__init__()
           self.topic = "object_position"
           self.rate = 10  

       def handle_message(self, msg):
           data = int(msg.data)
           pos = c.BodyGetPosition(box_body)
           if pos.z > 0:  
               z = pos.z
               if z > 1:  
                   self.last_position = data
               else:
                   self.ignore = True

       def handle_transform(self):
           trans = c.BodyGetTransform(box_body)
           msg = c.TransformStamped(
               header=c.Header(stamp=trans.header.stamp),
               child_frame=c.Vec3(trans.position.x, trans.position.y, trans.position.z),
               orientation=c.Vec3(trans.orientation.x, trans.orientation.y, trans.orientation.z),
               parent_frame=c.Vec3(trans.parent.position.x, trans.parent.position.y, trans.parent.position.z)
           )
           return msg

ros_manager = c.ROSManager()
   ros_manager.SetClockSource(c.ROSClockSource(system))
   ros_manager.SetBodyHandler(box_body)
   ros_manager.SetTransformHandler(box_body)
   ros_manager.RegisterHandler(self.topic, self)

while True:
       system.Advance(0.01)
       ros_manager.Update()
       if not self.ignore:
           self.publish_position()
       rospy.sleep(1 / self.rate)

rospy.spin()

import chrono as c
import numpy as np
import rospy
from std_msgs import Int32


g = 9.81
system = c.System(gx=0, gy=0, gz=-g)


floor_body = c.Body(name="floor", position=(0, 0, 0))
floor_size = c.Vec3(5, 5, 1)
c.BodyInitSize(floor_body, floor_size)
floor_material = c.Material(young=2e11, poisson=0.3)
c.BodySetMaterial(floor_body, floor_material)
system.AddBody(floor_body)


box_body = c.Body(name="box", mass=1, position=(2, 0, 1))
box_size = c.Vec3(1, 1, 1)
c.BodyInitSize(box_body, box_size)
box_material = c.Material(young=2e11, poisson=0.3)
c.BodySetMaterial(box_body, box_material)
system.AddBody(box_body)


c.BodyConnect(box_body, floor_body, connect_type=c.ConnectType.WHEEL)


class ObjectPositionHandler(c.ROSHandler):
    def __init__(self):
        super().__init__()
        self.topic = "object_position"
        self.rate = 10  
        self.ignore = False
        self.last_position = 0

    def handle_message(self, msg):
        if self.ignore:
            return
        data = int(msg.data)
        pos = c.BodyGetPosition(box_body)
        if pos.z > 0:  
            z = pos.z
            if z > 1:  
                self.last_position = data
            else:
                self.ignore = True

    def handle_transform(self):
        trans = c.BodyGetTransform(box_body)
        msg = c.TransformStamped(
            header=c.Header(stamp=trans.header.stamp),
            child_frame=c.Vec3(trans.position.x, trans.position.y, trans.position.z),
            orientation=c.Vec3(trans.orientation.x, trans.orientation.y, trans.orientation.z),
            parent_frame=c.Vec3(trans.parent.position.x, trans.parent.position.y, trans.parent.position.z)
        )
        return msg


ros_manager = c.ROSManager()
ros_manager.SetClockSource(c.ROSClockSource(system))
ros_manager.SetBodyHandler(box_body)
ros_manager.SetTransformHandler(box_body)
ros_handler = ObjectPositionHandler()
ros_manager.RegisterHandler(ros_handler.topic, ros_handler)


while True:
    system.Advance(0.01)
    ros_manager.Update()
    if not ObjectPositionHandler.ignore:
        pos = c.BodyGetPosition(box_body)
        if pos.z > 1:  
            msg = Int32()
            msg.data = int(pos.y)  
            rospy.publish(ros_handler.topic, msg, ros_handler.rate)
    rospy.sleep(1 / ros_handler.rate)


rospy.spin()