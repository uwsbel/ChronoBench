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











poon = veh.Poon()
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

print("Vehicle mass:               " + str(poon.GetVehicle().GetMass()))
print("Driveline type: " + poon.GetVehicle().GetDriveline().GetTemplateName())
print("Tire model:     " + poon.GetVehicle().GetTire(1, veh.LEFT).GetTemplateName())
print("\n")






ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True)
ground.SetPos(chrono.ChVector3d(0, 0, 0))
ground.SetFixed(True)
poisson_ratio = 0.03
ground.SetYoungModulus(2e7*(1-poisson_ratio)/poisson_ratio, True, True)
world.Add(ground)






driver = veh.ChInteractiveDriverIRR()
driver.SetWindowTitle('Poon')
driver.SetWindowSize(1280, 1024)
driver.SetChaseCamera(trackPoint, 6.0, 0.5)
driver.Initialize()
driver.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
driver.AddLightDirectional()
driver.AddSkyBox()
driver.AttachVehicle(poon.GetVehicle())


steering_time = 1.0  
throttle_time = 1.0  
braking_time = 0.3   
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)






print( "VEHICLE MASS: ",  poon.GetVehicle().GetMass())


render_steps = m.ceil(render_step_size / step_size)


step_number = 0
render_frame = 0

maxTrackingValue = 0

poon.GetVehicle().EnableRealtime(True)

while driver.Run() :

    time = poon.GetSystem().GetChTime()

    
    if (time >= tend):
        break

    
    if (step_number % render_steps == 0) :
        driver.BeginScene()
        driver.Render()
        driver.EndScene()
        render_frame += 1
    
    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(time)
    ground.Synchronize(time)
    poon.Synchronize(time, driver_inputs, ground)
    

    
    driver.Advance(step_size)
    ground.Advance(step_size)
    poon.Advance(step_size)
    

    
    step_number += 1