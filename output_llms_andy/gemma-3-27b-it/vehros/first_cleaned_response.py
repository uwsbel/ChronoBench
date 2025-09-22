import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.self_collision as sc
import pychrono.irrlicht as irrl
import time
import numpy as np


try:
    import rospy
    from std_msgs.msg import Float64
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import JointState
except ImportError:
    print("ROS is not installed. ROS functionality will be disabled.")
    rospy = None


class HMMWV_ROS_Sim:
    def __init__(self):
        
        self.system = chrono.ChSystemNSC()
        self.system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

        
        self.system.SetContactMethod(chrono.ChContactMethod.SCONTACT)
        self.system.SetContactNSC(chrono.ChContactNSC.NSC_SAT)
        self.system.SetContactSurfaceParameters(0.1, 0.1, 10000)

        
        self.terrain = chrono.ChTerrain()
        self.system.Add(self.terrain)
        self.terrain.SetTexture(chrono.GetChronoDataPath() + "terrain/textures/grass.png", 20, 20)
        self.terrain.SetHeightDistribution(chrono.ChFunction_Const(0), (-100, -100), (100, 100))
        self.terrain.SetFriction(0.5)
        self.terrain.SetRestitution(0.1)

        
        self.vehicle = veh.HMMWV()
        self.vehicle.SetVehicleModel(veh.VehicleModel.HMMWV)
        self.vehicle.Initialize(self.system)
        self.vehicle.SetEngineType(veh.EngineModel.SIMPLE)
        self.vehicle.SetTireModel(veh.TireModel.RADIAL)
        self.vehicle.SetTireForceModel(veh.TireForceModel.PACJ)
        self.vehicle.SetBrakeLock(True)
        self.vehicle.SetSuspensionLength(0.2)

        
        self.vehicle.SetPos(chrono.ChVectorD(0, 1, 0))

        
        self.driver = veh.ChDriver()
        self.driver.SetInput(veh.DriverInput.SPEED, 0)
        self.driver.SetInput(veh.DriverInput.STEERING, 0)
        self.driver.SetInput(veh.DriverInput.BRAKE, 0)
        self.vehicle.SetDriver(self.driver)

        
        self.ros_manager = None
        if rospy is not None:
            rospy.init_node('hmmwv_simulation', anonymous=True)
            self.ros_manager = ROSManager(self.vehicle)
            self.clock_sync_rate = rospy.Rate(100)  

    def run_simulation(self):
        
        while True:
            
            if self.ros_manager:
                self.ros_manager.synchronize()

            
            self.system.DoStepDynamics(0.001)

            
            if self.ros_manager:
                self.ros_manager.update()
                self.clock_sync_rate.sleep()

            
            if self.driver:
                self.driver.SetInput(veh.DriverInput.SPEED, 0.5)  
                self.driver.SetInput(veh.DriverInput.STEERING, 0)  
                self.driver.SetInput(veh.DriverInput.BRAKE, 0)  

            
            
            if self.system.GetChTime() > 10:  
                break


class ROSManager:
    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.pub_speed = None
        self.pub_steering = None
        self.pub_brake = None
        self.pub_joint_states = None
        self.subscriber_speed = None
        self.subscriber_steering = None

        if rospy is not None:
            self.pub_speed = rospy.Publisher('/hmmwv/speed', Float64, queue_size=10)
            self.pub_steering = rospy.Publisher('/hmmwv/steering', Float64, queue_size=10)
            self.pub_brake = rospy.Publisher('/hmmwv/brake', Float64, queue_size=10)
            self.pub_joint_states = rospy.Publisher('/hmmwv/joint_states', JointState, queue_size=10)

            self.subscriber_speed = rospy.Subscriber('/hmmwv/desired_speed', Float64, self.callback_speed)
            self.subscriber_steering = rospy.Subscriber('/hmmwv/desired_steering', Float64, self.callback_steering)

            self.desired_speed = 0.0
            self.desired_steering = 0.0

    def synchronize(self):
        if rospy is not None:
            rospy.spinOnce()

    def update(self):
        if rospy is not None:
            
            speed = self.vehicle.GetForwardSpeed()
            steering = self.vehicle.GetSteeringAngle()
            brake = self.vehicle.GetBrake()

            self.pub_speed.publish(speed)
            self.pub_steering.publish(steering)
            self.pub_brake.publish(brake)

            
            joint_states = JointState()
            joint_states.name = ['wheel_fl', 'wheel_fr', 'wheel_rl', 'wheel_rr']  
            joint_states.position = [0.0, 0.0, 0.0, 0.0]  
            self.pub_joint_states.publish(joint_states)

    def callback_speed(self, data):
        self.desired_speed = data.data

    def callback_steering(self, data):
        self.desired_steering = data.data


if __name__ == "__main__":
    sim = HMMWV_ROS_Sim()
    sim.run_simulation()