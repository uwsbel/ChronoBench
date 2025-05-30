import pychrono
import numpy as np
import rospy
from std_msgs.msg import Float64



vehicle_id = 1  
vehicle_type = "M1"  
contact_method = "Static"  
engine_type = "Diesel"
tire_model = "200/70" 
terrain_friction = 0.95  
terrain_restitution = 0.85  



terrain = np.zeros((10, 10))
terrain[0:5, 0:5] = 1.0  
terrain[5:10, 5:10] = 1.0


rospy_client = rospy.Client()
rospy_client.subscribe("vehicle_control", 1, callback=vehicle_control_callback)
rospy_client.spin()  



def vehicle_control_callback(data):
    
    try:
        
        vehicle_state = rospy.get_listener().get_vehicle_state()
        print(f"Vehicle State: {vehicle_state}")

        
        vehicle_state.update(data) 

        
        
        if data.payload == "accelerate":
            vehicle_state.update(np.array([1.0, 0.0, 0.0]))  
        elif data.payload == "brake":
            vehicle_state.update(np.array([0.0, -1.0, 0.0])) 
        elif data.payload == "turn":
            vehicle_state.update(np.array([0.0, 0.0, 1.0])) 
        else:
            vehicle_state.update(np.array([0.0, 0.0, 0.0])) 

    except rospy.ROSInterruptException:
        pass 



rospy.init_node('hmmwvSimulation')

simulation = pychrono.Simulation()
simulation.add_system("vehicle")  
simulation.add_sensor("vehicle", "velocity") 
simulation.add_actuator("vehicle", "engine") 
simulation.add_sensor("vehicle", "tire_pressure") 


try:
    while not rospy.is_shutdown():
        vehicle_control_callback()
        simulation.update() 

except rospy.ROSInterruptException:
    pass