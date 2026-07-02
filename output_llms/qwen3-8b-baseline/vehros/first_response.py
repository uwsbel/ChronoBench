import pychrono as chrono
from pychrono.vehicle import HMMWV
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

# Initialize PyChrono environment
system = chrono.ChSystemNSC()
system.SetGravity(chrono.ChVectorD(0, 0, -9.81))
system.SetContactMethod(chrono.ChContactMethod_SMC)
system.SetSolverType(chrono.ChSolver.Type_PGS)
system.SetSolverMaxIterations(100)
system.SetTimestep(1e-3)

# Create HMMWV vehicle
vehicle = HMMWV.HMMWV()
vehicle.SetContactMethod(chrono.ChContactMethod_SMC)
vehicle.SetEngineType(HMMWV.EngineType_TORQUE)
vehicle.SetTireType(HMMWV.TireType_RIGID)
vehicle.SetTireFrictionCoefficient(1.0)
vehicle.SetTireRestitution(0.3)
vehicle.Initialize(chrono.ChVectorD(0, 0, 0.5), chrono.ChQuaternionD(1, 0, 0, 0), chrono.ChVectorD(0, 0, 0))
system.Add(vehicle.GetChassisBody())

# Create terrain
terrain = chrono.ChBodyEasyBox(100, 100, 1, 1000, True, False)
terrain.SetPos(chrono.ChVectorD(0, 0, 0))
terrain.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))
terrain.SetMaterializedDensity(0)
terrain.SetMaterialize(False)
terrain.SetFriction(0.8)
terrain.SetRestitution(0.3)
system.Add(terrain)

# Initialize ROS node
rospy.init_node('hmmwv_sim', anonymous=True)
pub_vehicle_state = rospy.Publisher('vehicle_state', JointState, queue_size=10)
pub_terrain_state = rospy.Publisher('terrain_state', JointState, queue_size=10)

# Driver setup
driver = vehicle.GetDriver()
driver.SetThrottle(1.0)
driver.SetSteering(0.0)
driver.SetBraking(0.0)

def driver_callback(msg):
    driver.SetThrottle(msg.data[0])
    driver.SetSteering(msg.data[1])
    driver.SetBraking(msg.data[2])

sub_driver_input = rospy.Subscriber('driver_input', JointState, driver_callback)

def publish_vehicle_state():
    state_msg = JointState()
    state_msg.name = ['chassis', 'front_left', 'front_right', 'rear_left', 'rear_right']
    chassis_pos = vehicle.GetChassisBody().GetPos()
    chassis_vel = vehicle.GetChassisBody().GetPos_dt()
    # Placeholder for other parts' positions/velocities
    state_msg.position = [chassis_pos.z, 0, 0, 0, 0]
    state_msg.velocity = [chassis_vel.z, 0, 0, 0, 0]
    pub_vehicle_state.publish(state_msg)

def publish_terrain_state():
    state_msg = JointState()
    state_msg.name = ['terrain']
    state_msg.position = [terrain.GetPos().z]
    state_msg.velocity = [terrain.GetPos_dt().z]
    pub_terrain_state.publish(state_msg)

# Simulation loop
while not rospy.is_shutdown():
    # Advance the system
    system.DoStepDynamics(1e-3)
    
    # Update ROS publishers
    publish_vehicle_state()
    publish_terrain_state()
    
    # Synchronize with ROS clock
    rospy.sleep(0.01)