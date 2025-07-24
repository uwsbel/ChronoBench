import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math as m






veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


chassis_vis_type = veh.VisualizationType_MESH
suspension_vis_type = veh.VisualizationType_PRIMITIVES
steering_vis_type = veh.VisualizationType_PRIMITIVES
wheel_vis_type = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_NONE


tire_model = veh.TireModelType_TMEASY





inertia_method = veh.InertiaEstimationMethod_NONE


out_dir = "./"


step_size = 1e-3
tire_step_size = step_size


tend = 1000


render_step_size = 1.0 / 50  










car = veh.Car(veh.DataFile("vehicle/prius/prius.json"), 
               veh.DataFile("vehicle/prius/prius_chassis_mesh.json"), 
               veh.DataFile("vehicle/prius/prius_suspension_mesh.json"), 
               veh.DataFile("vehicle/prius/prius_wheel_mesh.json"), 
               veh.DataFile("vehicle/tires/prius_tire.json"))
car.SetChassisCollisionType(chassis_collision_type)
car.SetChassisFixed(False)
car.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
car.SetTireType(tire_model)
car.SetTireStepSize(tire_step_size)
car.SetMaxMotorVoltageRatio(0.16)
car.SetStallTorque(0.3)
car.SetTireRollingResistance(0.06)
car.Initialize()

car.SetChassisVisualizationType(chassis_vis_type)
car.SetSuspensionVisualizationType(suspension_vis_type)
car.SetSteeringVisualizationType(steering_vis_type)
car.SetWheelVisualizationType(wheel_vis_type)

car.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


track = veh.RigidTrack(car.GetSystem(), 
                       veh.DataFile("vehicle/track/straight2.json"))
track.Initialize()


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Prius')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 0.75), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(car.GetVehicle())


driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(0.02)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.06)
driver.Initialize()


steering_time = 1.0  
throttle_time = 1.0  
braking_time = 0.3   
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)






print( "VEHICLE MASS: ",  car.GetVehicle().GetMass())


render_steps = m.ceil(render_step_size / step_size)


step_number = 0
render_frame = 0

car.GetVehicle().EnableRealtime(True)

while vis.Run() :
    time = car.GetSystem().GetChTime()

    
    if (time >= tend):
        break

    
    if (step_number % render_steps == 0) :
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1
    
    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(time)
    track.Synchronize(time)
    car.Synchronize(time, driver_inputs, track)
    vis.Synchronize(time, driver_inputs)

    
    driver.Advance(step_size)
    track.Advance(step_size)
    car.Advance(step_size)
    vis.Advance(step_size)

    
    step_number += 1