import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sensor
import pychrono.postprocess as postprocess
import time
import numpy as np


try:
    import rospy
    from std_msgs.msg import Float64
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import JointState
except ImportError:
    print("ROS is not installed. ROS functionalities will be disabled.")
    rospy = None


class HMMWV_ROS_Simulation:
    def __init__(self):
        self.chrono_system = None
        self.vehicle = None
        self.terrain = None
        self.driver = None
        self.ros_manager = None

        
        self.timestep = 0.01
        self.simulation_duration = 60  
        self.current_time = 0.0

        
        if rospy is not None:
            rospy.init_node('hmmwv_simulation', anonymous=True)
            self.ros_pub_steering = rospy.Publisher('/hmmwv/steering', Float64, queue_size=10)
            self.ros_pub_throttle = rospy.Publisher('/hmmwv/throttle', Float64, queue_size=10)
            self.ros_pub_brake = rospy.Publisher('/hmmwv/brake', Float64, queue_size=10)
            self.ros_sub_steering = rospy.Subscriber('/hmmwv/steering_cmd', Float64, self.steering_callback)
            self.ros_sub_throttle = rospy.Subscriber('/hmmwv/throttle_cmd', Float64, self.throttle_callback)
            self.ros_sub_brake = rospy.Subscriber('/hmmwv/brake_cmd', Float64, self.brake_callback)

            self.steering_cmd = 0.0
            self.throttle_cmd = 0.0
            self.brake_cmd = 0.0
        else:
            print("ROS is not available.  Running in standalone mode.")
            self.steering_cmd = 0.0
            self.throttle_cmd = 0.0
            self.brake_cmd = 0.0

    def initialize(self):
        
        self.chrono_system = chrono.ChSystemNSC()
        self.chrono_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

        
        self.terrain = veh.Terrain()
        self.terrain.Create(self.chrono_system, 'rigid', 100, 100, 10, 10)  
        self.terrain.SetFriction(0.8)
        self.terrain.SetRestitution(0.1)

        
        self.vehicle = veh.HMMWV()
        self.vehicle.SetVehicleModel(veh.VehicleModel.HMMWV)
        self.vehicle.Initialize(self.chrono_system, self.terrain, veh.VehicleModel.HMMWV)

        
        self.vehicle.SetContactMethod(veh.VehicleContactMethod.SMC)

        
        self.vehicle.SetEngineType(veh.EngineType.GASOLINE)

        
        self.vehicle.SetTireModel(veh.TireModel.FTIRE)

        
        self.driver = veh.SimpleDriver()
        self.driver.SetVehicle(self.vehicle)

        
        if rospy is not None:
            self.ros_manager = ROSManager(self.vehicle)
        else:
            self.ros_manager = None

    def steering_callback(self, msg):
        self.steering_cmd = msg.data

    def throttle_callback(self, msg):
        self.throttle_cmd = msg.data

    def brake_callback(self, msg):
        self.brake_cmd = msg.data

    def run_simulation(self):
        if self.ros_manager:
            self.ros_manager.start()  

        while self.current_time < self.simulation_duration:
            
            self.driver.SetSteering(self.steering_cmd)
            self.driver.SetThrottle(self.throttle_cmd)
            self.driver.SetBrake(self.brake_cmd)

            
            self.chrono_system.DoStepDynamics(self.timestep)

            
            if self.ros_manager:
                self.ros_manager.update()

            self.current_time += self.timestep
            

        if self.ros_manager:
            self.ros_manager.stop()  
            rospy.signal_shutdown("Simulation finished")



class ROSManager:
    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.rate = rospy.Rate(100)  
        self.running = False

    def start(self):
        self.running = True
        import threading
        self.thread = threading.Thread(target=self.run)
        self.thread.daemon = True  
        self.thread.start()

    def stop(self):
        self.running = False
        self.thread.join()

    def run(self):
        while self.running:
            self.publish_vehicle_state()
            self.rate.sleep()

    def publish_vehicle_state(self):
        
        pos = self.vehicle.GetBody().GetPos()
        rot = self.vehicle.GetBody().GetRot()
        wheel_states = self.vehicle.GetWheelStates()

        
        joint_state = JointState()
        joint_state.name = ["wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr"]
        joint_state.position = [wheel_states[0].wheel_pos, wheel_states[1].wheel_pos, wheel_states[2].wheel_pos, wheel_states[3].wheel_pos]
        joint_state.velocity = [wheel_states[0].wheel_vel, wheel_states[1].wheel_vel, wheel_states[2].wheel_vel, wheel_states[3].wheel_vel]

        
        if rospy is not None:
            rospy.Publisher('/hmmwv/joint_states', JointState, queue_size=10).publish(joint_state)



if __name__ == "__main__":
    simulation = HMMWV_ROS_Simulation()
    simulation.initialize()
    simulation.run_simulation()
    print("Simulation completed.")