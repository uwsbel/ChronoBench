import chrono
import numpy as np
import pychrono
import roscat
import rospy
from pychrono.robot import ChRobot


try:
    rospy.init_node('chrono_hmmwv')
    rospy.init_interface()
    print("PyChrono environment initialized.")
except rospy.exceptions.ROS_NoErrors as e:
    print(f"Error initializing PyChrono: {e}")
    rospy.shutdown()




vehicle_name = "HMMWV_Vehicle"
vehicle_type = "HMMWV"
vehicle_radius = 1.0
vehicle_length = 5.0
vehicle_width = 2.0
vehicle_mass = 1000.0
vehicle_friction = 0.5
vehicle_restitution = 0.8
vehicle_tire_model = "Standard"


terrain_name = "Terrain"
terrain_radius = 1.0
terrain_length = 10.0
terrain_width = 5.0
terrain_friction = 0.2
terrain_restitution = 0.7


try:
    my_system = chrono.ChRobot(vehicle_name, vehicle_type, vehicle_radius, vehicle_length, vehicle_width, vehicle_mass, vehicle_friction, vehicle_restitution, vehicle_tire_model)
    print(f"Robot system initialized: {my_system}")
except Exception as e:
    print(f"Error initializing Robot system: {e}")
    my_system = None


try:
    collision_system = chrono.ChCollisionSystem.Type_BULLET
    my_system.SetCollisionSystemType(collision_system)
    print(f"Collision system initialized: {my_system}")
except Exception as e:
    print(f"Error initializing Collision system: {e}")


try:
    contact_system = chrono.ChContactMaterialNSC()
    my_system.SetContactMaterial(contact_system)
    print(f"Contact Material initialized: {my_system}")
except Exception as e:
    print(f"Error initializing Contact Material: {e}")


try:
    vis = chrono.ChVisualSystemIrrlicht()
    vis.AttachSystem(my_system)
    print("Visualization system attached.")
except Exception as e:
    print(f"Error attaching Visualization system: {e}")


try:
    body = my_system.GetBody()
    print(f"Body initialized: {body}")
except Exception as e:
    print(f"Error initializing Body: {e}")


try:
    
    rev_joint = my_system.GetJoint(0)
    rev_joint.SetRotation(chrono.ChVector3d(0, 0, 0))
    print("Revolute Joint initialized.")

    
    prismatic_joint = my_system.GetJoint(1)
    prismatic_joint.SetRotation(chrono.ChVector3d(0, 0, 0))
    print("Prismatic Joint initialized.")

    
    spherical_joint = my_system.GetJoint(2)
    spherical_joint.SetRotation(chrono.ChVector3d(0, 0, 0))
    print("Spherical Joint initialized.")

    
    universal_joint = my_system.GetJoint(3)
    universal_joint.SetRotation(chrono.ChVector3d(0, 0, 0))
    print("Universal Joint initialized.")

    
    rospy.init_node('chrono_hmmwv')
    rospy.loginfo("Simulation loop started.")
    while not rospy.is_shutdown():
        
        
        
        vehicle_position = my_system.GetPosition()
        vehicle_position += chrono.ChVector3d(0.1, 0.1, 0.1) 
        print(f"Vehicle position: {vehicle_position}")

        
        terrain_position = my_system.GetPosition()
        terrain_position += chrono.ChVector3d(0.01, 0.01, 0.01) 
        print(f"Terrain position: {terrain_position}")

        
        
        driver_position = my_system.GetPosition()
        driver_position += chrono.ChVector3d(0.05, 0.05, 0.05) 
        print(f"Driver position: {driver_position}")

        
        vis.Update()
        print("Visualization updated.")

        
        if rospy.is_shutdown():
            print("Simulation shutdown.")
            break

        
        time_step = chrono.ChTime(time_step)
        print(f"Time step: {time_step}")

        
        rospy.sleep(1/60) 

except rospy.exceptions.ROS_NoErrors as e:
    print(f"Error: {e}")
    print("Retrying...")
    rospy.sleep(1)  
except Exception as e:
    print(f"An unexpected error occurred: {e}")
    rospy.sleep(1)