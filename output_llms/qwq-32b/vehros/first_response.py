import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Clock

# Simulation parameters
step_size = 0.01
vis = True  # Set to False for headless simulation

# Initialize Chrono system
my_system = chrono.ChSystemNSC()
my_system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))

# Create terrain
terrain = veh.RigidTerrain(my_system)
ground_mat = chrono.ChMaterialSurfaceNSC()
ground_mat.SetFriction(0.9)
ground_mat.SetRestitution(0.1)
terrain.AddPatch(chrono.ChVectorD(0, 0, -1), chrono.ChVectorD(0, 0, 1), 100, 100, ground_mat)
terrain.Initialize()

# Initialize HMMWV vehicle
vehicle = veh.HMMWV_vehicle(my_system)
vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1), chrono.QUNIT))
vehicle.SetEngineType(veh.HMMWVEngine())  # Specify engine type
vehicle.SetTireType(veh.PacejkaTire())    # Specify tire model
vehicle.Initialize()

# Create driver system
driver = veh.UserDriver()
vehicle.Initialize(True, driver)  # Attach driver to vehicle

# ROS integration setup
rospy.init_node('chrono_simulation')
clock_pub = rospy.Publisher('/clock', Clock, queue_size=10)
state_pub = rospy.Publisher('/vehicle/state', Pose, queue_size=10)

def steering_callback(msg):
    driver.SetSteering(msg.data)
def throttle_callback(msg):
    driver.SetThrottle(msg.data)

steering_sub = rospy.Subscriber('/vehicle/steering', Float64, steering_callback)
throttle_sub = rospy.Subscriber('/vehicle/throttle', Float64, throttle_callback)

# Visualization setup
if vis:
    vis_params = irr.ChVisualSystemIrrlicht()
    vis_params.SetWindowSize(1024, 768)
    vis_params.SetWindowTitle('HMMWV Simulation')
    vis_params.SetCameraPosition(chrono.ChVectorD(5, 5, 5))
    vis_system = irr.ChVisualSystemIrrlicht()
    vis_system.SetSystem(my_system)
    vis_system.Initialize()
    vis_system.AddLogo()
    vis_system.AddSkyBox()
    vis_system.AddGrid()
    vis_system.AddLightDirectional()
    terrain.InitializeRender(vis_system)
    vehicle.InitializeRender(vis_system)

# Simulation loop
while not rospy.is_shutdown():
    # Process ROS messages
    rospy.spinOnce()
    
    # Update vehicle and driver
    vehicle.Synchronize(driver)
    
    # Advance simulation step
    my_system.DoStepDynamics(step_size)
    
    # Publish clock and vehicle state
    current_time = rospy.Time.now()
    clock_pub.publish(Clock(clock=current_time))
    
    chassis = vehicle.GetChassis()
    chassis_pos = chassis.GetPos()
    chassis_rot = chassis.GetRot()
    state = Pose()
    state.position.x = chassis_pos.x
    state.position.y = chassis_pos.y
    state.position.z = chassis_pos.z
    state.orientation = chassis_rot.GetAsQuaternion()
    state_pub.publish(state)
    
    # Visualization update
    if vis:
        vis_system.BeginScene()
        vis_system.DrawAll()
        vis_system.EndScene()
    
    # Sleep to match simulation step rate
    rospy.sleep(step_size)