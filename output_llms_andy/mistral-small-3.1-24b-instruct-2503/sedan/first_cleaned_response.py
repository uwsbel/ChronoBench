import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import pychrono.postprocess as postprocess





mysystem = chrono.ChSystemNSC()
mysystem.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


ground = chrono.ChBodyEasyBox(10, 0.1, 10, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -0.05, 0))
ground.SetBodyFixed(True)
mysystem.Add(ground)


vehicle = veh.ChVehicle(veh.ChVehicleType::VEHICLE_4W)
vehicle.SetChassisBody(chrono.ChBodyEasyBox(1.8, 0.3, 4.5, 200, True, True))
vehicle.SetChassisVisualizationType(veh.ChVehicleVisualizationType::CH_VISUALIZATION_TYPE_MESH)
vehicle.SetChassisCollisionType(veh.ChCollisionType::CH_MESH_TYPE)
vehicle.SetChassisPos(chrono.ChVectorD(0, 0.2, 0))
vehicle.SetChassisOrientation(chrono.ChQuaternionD(1, 0, 0, 0))
mysystem.Add(vehicle.GetChassisBody())


wheel_radius = 0.35
wheel_width = 0.2
wheel_inertia = chrono.ChVectorD(0.25, 1.2, 0.25)
wheel_mass = 20
wheel_friction = 0.9
wheel_rolling_resistance = 0.01


wheel = veh.ChWheel()
wheel.SetRadius(wheel_radius)
wheel.SetWidth(wheel_width)
wheel.SetInertia(wheel_inertia)
wheel.SetMass(wheel_mass)
wheel.SetFriction(wheel_friction)
wheel.SetRollingResistance(wheel_rolling_resistance)
wheel.SetTireModel(veh.ChTireModelTMeasy())
wheel.SetVisualizationType(veh.ChWheelVisualizationType::CH_VIS_WHEEL_MESH)
wheel.SetCollisionType(veh.ChCollisionType::CH_MESH_TYPE)
wheel.SetPos(chrono.ChVectorD(0.9, wheel_radius, 1.3))
wheel.SetOrientation(chrono.ChQuaternionD(1, 0, 0, 0))
vehicle.AddWheel(wheel)


wheel = veh.ChWheel()
wheel.SetRadius(wheel_radius)
wheel.SetWidth(wheel_width)
wheel.SetInertia(wheel_inertia)
wheel.SetMass(wheel_mass)
wheel.SetFriction(wheel_friction)
wheel.SetRollingResistance(wheel_rolling_resistance)
wheel.SetTireModel(veh.ChTireModelTMeasy())
wheel.SetVisualizationType(veh.ChWheelVisualizationType::CH_VIS_WHEEL_MESH)
wheel.SetCollisionType(veh.ChCollisionType::CH_MESH_TYPE)
wheel.SetPos(chrono.ChVectorD(0.9, wheel_radius, -1.3))
wheel.SetOrientation(chrono.ChQuaternionD(1, 0, 0, 0))
vehicle.AddWheel(wheel)


wheel = veh.ChWheel()
wheel.SetRadius(wheel_radius)
wheel.SetWidth(wheel_width)
wheel.SetInertia(wheel_inertia)
wheel.SetMass(wheel_mass)
wheel.SetFriction(wheel_friction)
wheel.SetRollingResistance(wheel_rolling_resistance)
wheel.SetTireModel(veh.ChTireModelTMeasy())
wheel.SetVisualizationType(veh.ChWheelVisualizationType::CH_VIS_WHEEL_MESH)
wheel.SetCollisionType(veh.ChCollisionType::CH_MESH_TYPE)
wheel.SetPos(chrono.ChVectorD(-0.9, wheel_radius, 1.3))
wheel.SetOrientation(chrono.ChQuaternionD(1, 0, 0, 0))
vehicle.AddWheel(wheel)


wheel = veh.ChWheel()
wheel.SetRadius(wheel_radius)
wheel.SetWidth(wheel_width)
wheel.SetInertia(wheel_inertia)
wheel.SetMass(wheel_mass)
wheel.SetFriction(wheel_friction)
wheel.SetRollingResistance(wheel_rolling_resistance)
wheel.SetTireModel(veh.ChTireModelTMeasy())
wheel.SetVisualizationType(veh.ChWheelVisualizationType::CH_VIS_WHEEL_MESH)
wheel.SetCollisionType(veh.ChCollisionType::CH_MESH_TYPE)
wheel.SetPos(chrono.ChVectorD(-0.9, wheel_radius, -1.3))
wheel.SetOrientation(chrono.ChQuaternionD(1, 0, 0, 0))
vehicle.AddWheel(wheel)


mysystem.Add(vehicle.GetVehicleBody())




application = chronoirr.ChIrrApp(mysystem, 'BMW E90 Sedan Simulation', chronoirr.dimension2du(1280, 720))

application.AddTypicalSky()
application.AddTypicalLogo(chrono.GetChronoDataPath() + 'logo_chrono_red.jpg')
application.AddTypicalCamera(chrono.ChVectorD(0, 2, 3))
application.AddTypicalLights()
application.SetChaseCamera(vehicle.GetVehicleBody(), chrono.ChVectorD(0, 1, -3), chrono.ChVectorD(0, 1, 0))


application.AssetBind(vehicle.GetVehicleBody(), veh.ChVehicleVisualizationType::CH_VISUALIZATION_TYPE_MESH)
application.AssetBind(vehicle.GetWheelBody(0), veh.ChWheelVisualizationType::CH_VIS_WHEEL_MESH)
application.AssetBind(vehicle.GetWheelBody(1), veh.ChWheelVisualizationType::CH_VIS_WHEEL_MESH)
application.AssetBind(vehicle.GetWheelBody(2), veh.ChWheelVisualizationType::CH_VIS_WHEEL_MESH)
application.AssetBind(vehicle.GetWheelBody(3), veh.ChWheelVisualizationType::CH_VIS_WHEEL_MESH)




driver = veh.ChDriver()
driver.Initialize(vehicle, mysystem)
driver.SetSteeringController(veh.ChDriver::STEERING_CONTROLLER_DRIVER)
driver.SetThrottleController(veh.ChDriver::THROTTLE_CONTROLLER_DRIVER)
driver.SetBrakingController(veh.ChDriver::BRAKING_CONTROLLER_DRIVER)




application.SetTimestep(0.01)
application.SetTryRealTime(True)

while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()

application.GetDevice().closeDevice()