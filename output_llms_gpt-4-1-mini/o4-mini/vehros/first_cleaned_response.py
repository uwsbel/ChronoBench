import pychrono.core as chrono
import pychrono.vehicle as veh


import pychrono.ros2 as ros2




chrono.SetChronoDataPath("/path/to/chrono/data/")     





contact_method = chrono.ChMaterialSurface.SMC

vehicle = veh.HMMWV_Vehicle(contact_method=contact_method,
                            engine_type=veh.EngineModelType_SimpleMap,
                            transmission_type=veh.TransmissionModelType_SimpleMap,
                            tire_type=veh.TireModelType_Fiala)


init_loc = chrono.ChVectorD(0, 0, 1.0)
init_rot = chrono.Q_from_AngX(0)  
vehicle.Initialize(chrono.ChCoordsysD(init_loc, init_rot))


vehicle.SetChassisVisualizationType(veh.VisualizationType_Primitive)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_Primitive)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_Primitive)
vehicle.SetWheelVisualizationType(veh.VisualizationType_Primitive)




terrain = veh.RigidTerrain(vehicle.GetSystem())


patch = terrain.AddPatch(chrono.ChCoordsysD(chrono.ChVectorD(0,0,0), chrono.QUNIT),
                         chrono.ChVectorD(20, 20, 0),  
                         0.8,                            
                         0.1,                            
                         2e7,                            
                         0.3)                            

patch.SetColor(chrono.ChColor(0.3, 0.3, 0.3))
patch.SetTexture(chrono.GetChronoDataFile("concrete.jpg"), 4, 4)
terrain.Initialize()




driver = veh.ChDriver(vehicle)
driver.SetInputDelay(0.1)  




ros_node_name = "hmmwv_sim_node"
ros_manager = ros2.ChRosManager(ros_node_name)


def clock_publisher():
    msg = ros2.ClockMsg()
    msg.clock = vehicle.GetSystem().GetChTime()
    return msg

ros_manager.CreatePublisher("/clock", ros2.ClockMsg, clock_publisher,
                            topic_qos=ros2.QoSProfile(depth=1, reliable=True))



def driver_input_callback(msg):
    driver.SetThrottle(msg.throttle)
    driver.SetSteering(msg.steering)
    driver.SetBraking(msg.braking)

ros_manager.CreateSubscriber("/driver/input",
                             ros2.DriverInputMsg,
                             driver_input_callback,
                             topic_qos=ros2.QoSProfile(depth=1, reliable=True))


def vehicle_state_publisher():
    state = vehicle.ExportStateJSON()  
    msg = ros2.VehicleStateMsg()
    msg.json_state = state
    return msg

ros_manager.CreatePublisher("/vehicle/state",
                            ros2.VehicleStateMsg,
                            vehicle_state_publisher,
                            topic_qos=ros2.QoSProfile(depth=1, reliable=True))


ros_manager.Start()




t_end      = 10.0        
step_size  = 1.0/100.0   

system = vehicle.GetSystem()
time   = 0.0

while time < t_end:
    
    ros_manager.SpinOnce()

    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver.GetInputs(), terrain.GetForces())
    ros_manager.PublishAll()  

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)

    
    time += step_size


ros_manager.Shutdown()