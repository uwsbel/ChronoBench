import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import pychrono.sensor as sensor
import time
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

class HMMWV_ROS_Simulation:
    def __init__(self):
        
        chrono.SetChronoDataPath("./")  
        self.system = chrono.ChSystemNSC()
        self.system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

        
        self.vehicle = veh.HMMWV()
        self.vehicle.SetVehicleModel(veh.VehicleModel.HMMWV)
        self.vehicle.Initialize(self.system)

        
        self.vehicle.SetTireFormula(veh.TireModelType.MT)
        self.vehicle.SetEngineType(veh.EngineModelType.SIMPLE)
        self.vehicle.SetContactMethod(veh.ContactMethod.SWEEP_AND_CLIP)

        
        self.vehicle.SetInitialPosition(chrono.ChVectorD(0, 1, 0))
        self.vehicle.SetInitialVelocity(chrono.ChVectorD(0, 0, 0))

        
        self.terrain = veh.Terrain()
        self.terrain.CreateBox(self.system, chrono.ChVectorD(100, 0.1, 100), chrono.ChVectorD(0, -0.1, 0))
        self.terrain.SetFriction(0.8)
        self.terrain.SetRestitution(0.1)

        
        self.driver = veh.SimpleDriver()
        self.driver.SetInput(veh.DriverInput.STEERING, 0)
        self.driver.SetInput(veh.DriverInput.THROTTLE, 0)
        self.driver.SetInput(veh.DriverInput.BRAKE, 0)
        self.vehicle.SetDriver(self.driver)

        
        rospy.init_node('hmmwv_simulation', anonymous=True)
        self.ros_manager = veh.ROSInterface(self.system, self.vehicle)

        
        self.steering_pub = rospy.Publisher('/hmmwv/steering', Float64, queue_size=10)
        self.throttle_pub = rospy.Publisher('/hmmwv/throttle', Float64, queue_size=10)
        self.brake_pub = rospy.Publisher('/hmmwv/brake', Float64, queue_size=10)

        
        rospy.Subscriber('/hmmwv/cmd_vel', Twist, self.cmd_vel_callback)

        
        self.time_step = 0.01
        self.simulation_running = True

    def cmd_vel_callback(self, msg):
        
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        
        throttle = linear_x
        steering = angular_z * 0.5  
        brake = 0.0

        self.driver.SetInput(veh.DriverInput.THROTTLE, throttle)
        self.driver.SetInput(veh.DriverInput.STEERING, steering)
        self.driver.SetInput(veh.DriverInput.BRAKE, brake)

    def run_simulation(self):
        
        while self.simulation_running:
            
            self.ros_manager.Synchronize()

            
            self.system.DoStepDynamics(self.time_step)

            
            self.ros_manager.Update()

            
            self.steering_pub.publish(self.driver.GetInput(veh.DriverInput.STEERING))
            self.throttle_pub.publish(self.driver.GetInput(veh.DriverInput.THROTTLE))
            self.brake_pub.publish(self.driver.GetInput(veh.DriverInput.BRAKE))

            
            time.sleep(self.time_step)
            if self.system.GetChTime() > 30:
                self.simulation_running = False
                print("Simulation finished.")

if __name__ == "__main__":
    try:
        simulation = HMMWV_ROS_Simulation()
        simulation.run_simulation()
    except rospy.ROSInterruptException:
        pass