import chrono as chrono
from chrono import ChVector, ChFrame
import time
import roslib
import rospy
import numpy as np



sim = chrono.Simulation()


world = chrono.ChWorld()
sim.SetWorld(world)



hmmwv = chrono.ChBody()
hmmwv.SetName("HMMWV")
hmmwv.SetPos(ChVector(0, 0, 0))
hmmwv.SetMass(500)
hmmwv.SetInertia(ChVector(100, 100, 100))
hmmwv.SetBodyFixed(True)


terrain = chrono.ChBody()
terrain.SetName("Terrain")
terrain.SetPos(ChVector(0, -10, 0))
terrain.SetFriction(0.5)
terrain.SetRestitution(0.5)
terrain.SetBodyFixed(True)


driver = chrono.ChDriver()
driver.SetName("Driver")
driver.SetPos(ChVector(0, 0, 0))



hmmwv.SetContactMethod(chrono.ChContactMethod.COPLANAR)


hmmwv.SetEngineType(chrono.ChEngineType.VEHICLE_ENGINE)


hmmwv.SetTireModel(chrono.ChTireModel.VEHICLE_TIRE)


hmmwv.SetState(chrono.ChState.STOPPED)



roslib.load_manifest('pychrono_ros')
rospy.init_node('pychrono_ros_node', anonymous=True)


rospy.Timer(rospy.Duration(0.01), handler=chrono_ros_clock_synchronization)
rospy.Timer(rospy.Duration(0.01), handler=chrono_ros_driver_inputs)
rospy.Timer(rospy.Duration(0.01), handler=chrono_ros_vehicle_state)



def simulation_loop():
    
    sim.Step()

    
    ros_manager.update()


while not rospy.is_shutdown():
    simulation_loop()
    time.sleep(0.01)


def chrono_ros_clock_synchronization(event):
    
    chrono_ros_clock_synchronize()


def chrono_ros_driver_inputs(event):
    
    driver_inputs = get_driver_inputs()

    
    hmmwv.SetState(chrono.ChState.MOVING, driver_inputs)


def chrono_ros_vehicle_state(event):
    
    vehicle_state = get_vehicle_state()

    
    ros_manager.publish_vehicle_state(vehicle_state)


class ROSManager:
    def __init__(self):
        self.publisher = rospy.Publisher('vehicle_state', VehicleState, queue_size=10)

    def update(self):
        
        self.publisher.publish(get_vehicle_state())


class VehicleState:
    def __init__(self):
        self.position = ChVector(0, 0, 0)
        self.velocity = ChVector(0, 0, 0)
        self.acceleration = ChVector(0, 0, 0)


class DriverInputs:
    def __init__(self):
        self.steering_angle = 0
        self.accelerator = 0
        self.brake = 0


def get_driver_inputs():
    
    driver_inputs = DriverInputs()
    driver_inputs.steering_angle = rospy.get_param('steering_angle')
    driver_inputs.accelerator = rospy.get_param('accelerator')
    driver_inputs.brake = rospy.get_param('brake')
    return driver_inputs


def get_vehicle_state():
    
    vehicle_state = VehicleState()
    vehicle_state.position = hmmwv.GetPos()
    vehicle_state.velocity = hmmwv.GetVelocity()
    vehicle_state.acceleration = hmmwv.GetAcceleration()
    return vehicle_state


def chrono_ros_clock_synchronize():
    
    rospy.Time.sync()