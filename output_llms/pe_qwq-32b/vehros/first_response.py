import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import pychrono_ros as ros_utils
import rospy

# Initialize Chrono system with NSC contact method
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Gravity

# Create and initialize HMMWV vehicle
vehicle = veh.HMMWV()
vehicle.SetContactMethod(chrono.ChContactMethod.NSC)
vehicle.SetEngineType(veh.EngineType.V8)  # Specify engine type
vehicle.SetTireType(veh.TireModelType.PACEJKA)  # Specify tire model
vehicle.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1)),  # Position (z=1m to avoid ground penetration)
                  True, True)  # Enable visualization and collision

# Create terrain with specified material properties
terrain = veh.RigidTerrain(system)
material = chrono.ChMaterialSurfaceNSC()
material.SetFriction(0.8)  # Friction coefficient
material.SetRestitution(0.1)  # Restitution coefficient
patch = terrain.AddPatch(material, chrono.ChCoordsysD(), 100, 100)  # 100m x 100m patch
terrain.Initialize()

# Initialize driver system
driver = veh.HMMWVDriver()
vehicle.SetDriver(driver)

# ROS Integration Setup
rospy.init_node('hmmwv_simulation')
ros_manager = ros_utils.ChRosManager(system)
ros_manager.Initialize()

# ROS Handler Registration
def clock_callback(clock):
    system.SetChTime(clock.clock.rostime.to_sec())

ros_manager.RegisterClockCallback(clock_callback)

def input_callback(msg):
    driver.SetThrottle(msg.throttle)
    driver.SetSteering(msg.steering)

ros_manager.RegisterInputHandler('/vehicle/inputs', input_callback)

def state_publisher():
    state = {
        'position': vehicle.GetChassis().GetPos(),
        'velocity': vehicle.GetChassis().GetPos_dt(),
        'engine_rpm': vehicle.GetEngine().GetRotationSpeed()
    }
    ros_manager.PublishState(state, '/vehicle/state')

ros_manager.RegisterStatePublisher(state_publisher)

# Simulation Loop
while not rospy.is_shutdown():
    ros_manager.Synchronize()
    system.DoStepDynamics(ros_manager.GetTimeStep())
    driver.SynchronizeTime(ros_manager.GetChTime())
    vehicle.UpdateDriver()
    ros_manager.Process()
    ros_manager.Publish()