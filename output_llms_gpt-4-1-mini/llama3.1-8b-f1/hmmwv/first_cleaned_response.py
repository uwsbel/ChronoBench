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



inertia_estimation_method = veh.InertiaEstimationMethod_DIAGNOL_MOMENTS_OF_INERTIA


trackPoint = chrono.ChVector3d(0.0, 0.0, 1.75)


step_size = 1e-3
tire_step_size = step_size


tend = 15


render_step_size = 1.0 / 50  










driver = veh.ChDriver()
driver.SetVehicle(veh_cast)
driver.SetSteeringDelta(render_step_size / 3)
driver.SetThrottleDelta(render_step_size / 3)
driver.SetBrakingDelta(render_step_size / 3)
driver.Initialize()

tire_vis_type = veh.VisualizationType_MESH  

vehicle = veh.ChVehicleRigid(veh_cast.GetSystem())
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.Initialize()

vehicle.SetChassisVisualizationType(chassis_vis_type)
vehicle.SetSuspensionVisualizationType(suspension_vis_type)
vehicle.SetSteeringVisualizationType(steering_vis_type)
vehicle.SetWheelVisualizationType(wheel_vis_type)
vehicle.SetTireVisualizationType(tire_vis_type)

vehicle.Initialize()


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Rigid Body Jeep')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(veh_cast)


driver_system = veh.ChInteractiveSystemIRR(vis)
driver_system.SetChaseCamera(trackPoint, 6.0, 0.5)
driver_system.Initialize()






print( "VEHICLE MASS: ",  vehicle.GetMass())


render_steps = m.ceil(render_step_size / step_size)


step_number = 0
render_frame = 0

veh_cast.EnableRealtime(True)

while vis.Run() :
    time = veh_cast.GetChTime()

    
    if (time >= tend):
        break

    
    if (step_number % render_steps == 0) :
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1
    
    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs)
    vis.Synchronize(time, driver_inputs)

    
    driver.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    
    step_number += 1