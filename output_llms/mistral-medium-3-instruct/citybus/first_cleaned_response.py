import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.vehicle.wheeled_vehicle as veh_wheel
import numpy as np





step_size = 0.02  


t_end = 100





system = chrono.ChSystemNSC()


system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))


vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('CityBus Simulation')
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 0, 2))
vis.AddTypicalLights()
vis.AssetBindAll()
vis.AssetUpdateAll()


vehicle = veh_wheel.WheelVehicle(chrono.GetChronoDataFile('vehicle/hmmwv/vehicle.json'))
vehicle.Initialize(system, veh_wheel.VehicleSide.NORTH)


vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.5),
                                         chrono.ChQuaternionD(1, 0, 0, 0)))


vehicle.GetChassis().SetVisualizationType(veh_wheel.VisualizationType.MESH)
for axle in vehicle.GetAxles():
    for wheel in axle.GetWheels():
        wheel.SetVisualizationType(veh_wheel.VisualizationType.PRIM)





terrain = veh.RigidTerrain(system)


texture = veh.RigidTerrain.Texture(chrono.GetChronoDataFile('terrain/textures/tile4.jpg'), 20, 20)
terrain.AddTexture(texture)


patch = veh.RigidTerrain.Patch(chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 1),
                              100, 100, 0, 0, 100, 100)
terrain.AddPatch(patch)
terrain.Initialize()





driver = veh_wheel.ChInteractiveDriverIRR(vis.GetDevice())


driver.SetSteeringDelta(0.0)
driver.SetThrottleDelta(0.0)
driver.SetBrakingDelta(0.0)





num_steps = int(t_end / step_size)


step = 0


camera = vis.GetCamera()
camera.SetPosition(chrono.ChVectorD(0, -5, 2))
camera.SetAimPoint(chrono.ChVectorD(0, 0, 0.5))


while vis.Run() and step < num_steps:
    
    time = step * step_size

    
    driver_inputs = driver.GetInputs()

    
    vehicle.SetSteering(driver_inputs.m_steering)
    vehicle.SetThrottle(driver_inputs.m_throttle)
    vehicle.SetBraking(driver_inputs.m_braking)

    
    vehicle.Update(time)

    
    system.DoStepDynamics(step_size)

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    vehicle_pos = vehicle.GetChassis().GetPos()
    camera.SetPosition(chrono.ChVectorD(vehicle_pos.x, vehicle_pos.y - 5, 2))
    camera.SetAimPoint(chrono.ChVectorD(vehicle_pos.x, vehicle_pos.y, 0.5))

    
    step += 1