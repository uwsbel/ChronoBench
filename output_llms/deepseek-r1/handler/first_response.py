import pychrono as chrono
import pychrono.ros as chros
import rospy
from std_msgs.msg import Int32

class CustomROSPublisher(chros.ChROSHandler):
    def __init__(self):
        super().__init__()
        self.topic = "/chrono_counter"
        self.publisher = None
        self.counter = 0
        
    def Initialize(self):
        self.publisher = rospy.Publisher(self.topic, Int32, queue_size=10)
        return True
        
    def Tick(self, time):
        msg = Int32()
        msg.data = self.counter
        self.publisher.publish(msg)
        self.counter += 1
        return True

# Initialize ROS node
rospy.init_node('pychrono_ros_sim')

# Create Chrono system
system = chrono.ChSystemNSC()
system.Set_Grav(chrono.ChVectorD(0, 0, -9.81))

# Create material for contact
material = chrono.ChMaterialSurfaceNSC()
material.SetFriction(0.5)
material.SetRestitution(0.2)

# Create fixed floor
floor = chrono.ChBodyEasyBox(10, 10, 0.1, 1000, True, True, material)
floor.SetPos(chrono.ChVectorD(0, 0, -0.5))
floor.SetBodyFixed(True)
system.Add(floor)

# Create movable box
box = chrono.ChBodyEasyBox(1, 1, 1, 1000, True, True, material)
box.SetPos(chrono.ChVectorD(0, 0, 2))
system.Add(box)

# Setup ROS manager
manager = chros.ChROSManager()
manager.AttachSystem(system)

# Add handlers
manager.RegisterHandler(chros.ChROSClockHandler())
manager.RegisterHandler(chros.ChROSBodyHandler(system))
manager.RegisterHandler(chros.ChROSTFHandler(system, 10))  # 10 Hz update
manager.RegisterHandler(CustomROSPublisher())

# Initialize ROS manager
manager.Initialize()

# Simulation parameters
time_step = 0.01
end_time = 10.0
realtime_timer = chrono.ChRealtimeStepTimer()

# Main simulation loop
while system.GetChTime() < end_time and not rospy.is_shutdown():
    time = system.GetChTime()
    
    # Update ROS components
    manager.Update(time, time_step)
    
    # Advance simulation
    system.DoStepDynamics(time_step)
    
    # Maintain real-time pacing
    realtime_timer.Spin(time_step)

print("Simulation completed")