import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math
import os








veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVector3d(0, 0, 0.4)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


chassis_vis_type = veh.VisualizationType_MESH
suspension_vis_type = veh.VisualizationType_PRIMITIVES
steering_vis_type = veh.VisualizationType_PRIMITIVES
wheel_vis_type = veh.VisualizationType_MESH
tire_vis_type = veh.VisualizationType_MESH


tire_model = veh.TireModelType_PAC02


trackPoint = chrono.ChVector3d(0.0, 0.0, 1.75)


step_size = 1e-3
tire_step_size = step_size


tend = 1000


render_step_size = 1.0 / 50  


out_dir = "./GATOR"





print( "Copyright (c) 2017 projectchrono.org\n")






gator = veh.Gator()
gator.SetContactMethod(chrono.ChContactMethod_NSC)
gator.SetChassisFixed(False)
gator.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
gator.SetTireType(tire_model)
gator.SetTireStepSize(tire_step_size)
gator.SetInitFwdVel(0.0)
gator.Initialize()

gator.SetChassisVisualizationType(chassis_vis_type)
gator.SetSuspensionVisualizationType(suspension_vis_type)
gator.SetSteeringVisualizationType(steering_vis_type)
gator.SetWheelVisualizationType(wheel_vis_type)
gator.SetTireVisualizationType(tire_vis_type)

print("Vehicle mass:               " + str(gator.GetVehicle().GetMass()))
print("Driveline type: " + gator.GetVehicle().GetDriveline().GetTemplateName())
print("Tire type:    " + gator.GetTire(0, veh.LEFT).GetTemplateName())
print(" ")

gator.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)





terrain = veh.RigidTerrain(gator.GetSystem())
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, 
                         chrono.CSYSNORM, 
                         200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 1.0))
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
terrain.Initialize()






vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Gator')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(gator.GetVehicle())


driver = veh.ChInteractiveDriverIRR(vis)


steering_time = 1.0  
throttle_time = 1.0  
braking_time = 0.3   
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()




sens = veh.ChSensorManager(gator.GetVehicle())
intensity = 2.0
sens.PushFilter(veh.ChFilterVisualPower(intensity))

mp = sens.PushFilter(veh.ChFilterMultifilm())
sens.PushFilter(veh.ChFilterWavelength())

cam = sens.AddSensor(veh.ChCameraSensor(
    cam Parallel,
    update_rate,
    image_width,
    image_height,
    offset,
    cam_loc,
    cam_up
))
sens.AddPointLight(veh.ChPointLight(200, 200, 300, 16, 16))
sens.Initialize()





gator.GetVehicle().EnableRealtime(True)

while vis.Run() :
    time = gator.GetSystem().GetChTime()

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    driver_inputs = driver.GetInputs()

    
    sens.Update()

    
    terrain.Synchronize(time)
    terrain.Render()

    
    driver.Synchronize(time)
    driver_input = driver.GetInputs()

    
    gator.Synchronize(time, driver_inputs, terrain)
    gator.Advance(step_size)

chrono.ChCamera::SetLenMM(float fmm)