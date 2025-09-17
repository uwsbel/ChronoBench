import pychrono as chrono
import pychrono.collision as collision
import pychrono.geometry as geometry
import pychrono.dynamics as dynamics
import pychrono.materials as materials
import pychrono.ros as ros
import rospy
from pychrono.ros import ROSHandler


chrono.init()


dt = 0.01  
gravity = 9.81  


chrono_system = chrono.ChronoSystem()
chrono_system.setGravity(chrono.Vector3(0, -gravity, 0))


floor = geometry.Plane(chrono_system, geometry.Vector3(0, 0, 0), geometry.Vector3(0, 1, 0))
floor.setMaterial(materials.Material(materials.Elasticity(1.0)))


box = geometry.Box(chrono_system, geometry.Vector3(1, 1, 1), geometry.Vector3(0, 0, 0))
box.setMaterial(materials.Material(materials.Elasticity(0.5)))


box.setPosition(chrono.Vector3(0, 0, 0))
box.setLinearVelocity(chrono.Vector3(0, 0, 0))


ros_handler = ROSHandler(chrono_system)
ros_handler.setTopic("box_position")
ros_handler.setCallback(lambda: print("Box position updated"))


chrono_system.setClockHandler(ros_handler)
chrono_system.setBodyHandler(ros_handler)
chrono_system.setTransformHandler(ros_handler)


chrono_system.registerHandler(ros_handler)


while True:
    
    chrono_system.advance(dt)

    
    ros_handler.update()

    
    rospy.sleep(1/100)