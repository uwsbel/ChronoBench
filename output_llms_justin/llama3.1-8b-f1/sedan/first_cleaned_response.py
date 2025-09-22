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


inertia_method = veh.InertiaEstimationMethod_MESH_PRIMITIVES


trackPoint = chrono.ChVector3d(0.0, 0.0, 1.75)


step_size = 1e-3
tire_step_size = step_size


tend = 15


render_step_size = 1.0 / 50  


out_dir = "./POOPoon"










poon = veh.PoonChassis()
poon.SetContactMethod(chrono.ChContactMethod_NSC)
poon.SetChassisCollisionType(chassis_collision_type)
poon.SetChassisFixed(False)
poon.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
poon.SetTireType(tire_model)
poon.SetTireStepSize(tire_step_size)
poon.SetInertiaEstimationMethod(inertia_method)
poon.Initialize()

poon.SetChassisVisualizationType(chassis_vis_type)
poon.SetSuspensionVisualizationType(suspension_vis_type)
poon.SetSteeringVisualizationType(steering_vis_type)
poon.SetWheelVisualizationType(wheel_vis_type)

poon.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


tire_vis = veh.TireVisualization()
tire_vis.SetLineThickness(0.02)
tire_vis.SetHubRadius(0.01)
tire_vis.Initialize()
poon.GetChassis().AddVisualization(tire_vis)


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Poon chassis')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(poon.GetVehicle())


driver = veh.ChInteractiveDriverIRR(vis)


steering_time = 1.0  
throttle_time = 1.0  
braking_time = 0.3   
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()






print( "VEHICLE MASS: ",  pois.GetMass())


render_steps = m.ceil(render_step_size / step_size)


step_number = 0
render_frame = 0

poon.GetVehicle().EnableRealtime(True)

while vis.Run() :
    time = pooch.GetSystem().GetChTime()

    
    if (time >= tend):
        break

    
    if (step_number % render_steps == 0) :
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1
    
    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(time)
    vis.Synchronize(time, driver_inputs)
    poon.Synchronize(time, driver_inputs)

    
    driver.Advance(step_size)
    vis.Advance(step_size)
    poon.Advance(step_size)

    
    step_number += 1