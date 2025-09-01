import os
import rospy
from geometry_msgs.msg import Twist, PoseStamped
import pychrono as chrono
import pychrono.vehicle as veh

# Global variables for ROS callbacks
current_steering = 0.0
current_throttle = 0.0
current_brake = 0.0

def cmd_vel_callback(msg):
    global current_steering, current_throttle, current_brake
    current_steering = max(-1.0, min(1.0, msg.angular.z))
    current_throttle = max(0.0, min(1.0, msg.linear.x))
    current_brake = max(0.0, min(1.0, -msg.linear.y))

def main():
    rospy.init_node('chrono_simulation')

    # Set Chrono data path
    chrono.SetChronoDataPath(os.path.join(os.environ['CHRONO_DATA_DIR'], 'data/'))
    
    # Initialize Chrono system
    system = chrono.ChSystemSMC()
    system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))
    system.SetSolverMaxIterations(100)

    # Create and initialize HMMWV vehicle
    vehicle = veh.HMMWV(system)
    vehicle.SetContactMethod(chrono.ChContactMethod_SMC)
    vehicle.SetChassisFixed(False)
    vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1), chrono.QUNIT))
    vehicle.SetEngineType(veh.EngineModelType_SHAFTS)
    vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
    vehicle.SetDriveType(veh.DrivelineTypeWV_AWD)
    vehicle.SetTireType(veh.TireModelType_TMEASY)
    vehicle.Initialize()

    # Set vehicle visualization parameters
    vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)

    # Create and initialize terrain
    terrain = veh.RigidTerrain(system)
    patch = terrain.AddPatch(
        chrono.ChCoordsysD(), 
        chrono.ChVectorD(200, 200, 0.5)
    )
    patch.SetContactFriction(0.8)
    patch.SetContactRestitution(0.01)
    patch.SetTexture(veh.TerrainTextureType_GRASS, 64, 64)
    terrain.Initialize()

    # ROS communication setup
    rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback)
    state_pub = rospy.Publisher('/vehicle_state', PoseStamped, queue_size=10)

    # Simulation parameters
    time_step = 0.01
    time = 0.0

    while not rospy.is_shutdown():
        # Prepare driver inputs
        driver_inputs = veh.DriverInputs()
        driver_inputs.m_steering = current_steering
        driver_inputs.m_throttle = current_throttle
        driver_inputs.m_braking = current_brake

        # Synchronize vehicle and terrain
        vehicle.Synchronize(time, driver_inputs)
        terrain.Synchronize(time)

        # Advance simulation
        system.DoStepDynamics(time_step)
        time += time_step

        # Publish vehicle state
        chassis = vehicle.GetChassisBody()
        pos = chassis.GetPos()
        rot = chassis.GetRot()

        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "world"
        pose_msg.pose.position.x = pos.x
        pose_msg.pose.position.y = pos.y
        pose_msg.pose.position.z = pos.z
        pose_msg.pose.orientation.x = rot.e0
        pose_msg.pose.orientation.y = rot.e1
        pose_msg.pose.orientation.z = rot.e2
        pose_msg.pose.orientation.w = rot.e3

        state_pub.publish(pose_msg)

        # Process ROS callbacks
        rospy.spin_once()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass