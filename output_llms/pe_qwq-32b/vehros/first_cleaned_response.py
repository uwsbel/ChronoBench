import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import pychrono_ros as ros_utils
import rospy


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


vehicle = veh.HMMWV()
vehicle.SetContactMethod(chrono.ChContactMethod.NSC)
vehicle.SetEngineType(veh.EngineType.V8)  
vehicle.SetTireType(veh.TireModelType.PACEJKA)  
vehicle.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1)),  
                  True, True)  


terrain = veh.RigidTerrain(system)
material = chrono.ChMaterialSurfaceNSC()
material.SetFriction(0.8)  
material.SetRestitution(0.1)  
patch = terrain.AddPatch(material, chrono.ChCoordsysD(), 100, 100)  
terrain.Initialize()


driver = veh.HMMWVDriver()
vehicle.SetDriver(driver)


rospy.init_node('hmmwv_simulation')
ros_manager = ros_utils.ChRosManager(system)
ros_manager.Initialize()


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


while not rospy.is_shutdown():
    ros_manager.Synchronize()
    system.DoStepDynamics(ros_manager.GetTimeStep())
    driver.SynchronizeTime(ros_manager.GetChTime())
    vehicle.UpdateDriver()
    ros_manager.Process()
    ros_manager.Publish()