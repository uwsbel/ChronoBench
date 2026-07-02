import pychrono as chrono
from pychrono.vehicle import HMMWV
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


system = chrono.ChSystemNSC()
system.SetGravity(chrono.ChVectorD(0, 0, -9.81))
system.SetContactMethod(chrono.ChContactMethod_SMC)
system.SetSolverType(chrono.ChSolver.Type_PGS)
system.SetSolverMaxIterations(100)
system.SetTimestep(1e-3)


vehicle = HMMWV.HMMWV()
vehicle.SetContactMethod(chrono.ChContactMethod_SMC)
vehicle.SetEngineType(HMMWV.EngineType_TORQUE)
vehicle.SetTireType(HMMWV.TireType_RIGID)
vehicle.SetTireFrictionCoefficient(1.0)
vehicle.SetTireRestitution(0.3)
vehicle.Initialize(chrono.ChVectorD(0, 0, 0.5), chrono.ChQuaternionD(1, 0, 0, 0), chrono.ChVectorD(0, 0, 0))
system.Add(vehicle.GetChassisBody())


terrain = chrono.ChBodyEasyBox(100, 100, 1, 1000, True, False)
terrain.SetPos(chrono.ChVectorD(0, 0, 0))
terrain.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))
terrain.SetMaterializedDensity(0)
terrain.SetMaterialize(False)
terrain.SetFriction(0.8)
terrain.SetRestitution(0.3)
system.Add(terrain)


rospy.init_node('hmmwv_sim', anonymous=True)
pub_vehicle_state = rospy.Publisher('vehicle_state', JointState, queue_size=10)
pub_terrain_state = rospy.Publisher('terrain_state', JointState, queue_size=10)


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
    
    state_msg.position = [chassis_pos.z, 0, 0, 0, 0]
    state_msg.velocity = [chassis_vel.z, 0, 0, 0, 0]
    pub_vehicle_state.publish(state_msg)

def publish_terrain_state():
    state_msg = JointState()
    state_msg.name = ['terrain']
    state_msg.position = [terrain.GetPos().z]
    state_msg.velocity = [terrain.GetPos_dt().z]
    pub_terrain_state.publish(state_msg)


while not rospy.is_shutdown():
    
    system.DoStepDynamics(1e-3)
    
    
    publish_vehicle_state()
    publish_terrain_state()
    
    
    rospy.sleep(0.01)