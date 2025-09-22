import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math
import os








veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVector3d(0, 0, 0.4)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


chassis_vis_type = veh.VisualizationType_MESH
suspension_vis_type = veh.VisualizationType_PRIMITIVES
steering_vis_type = veh.VisualizationType_PRIMITIVES
wheel_vis_type = veh.VisualizationType_MESH


trackPoint = chrono.ChVector3d(0.0, 0.0, 1.75)


step_size = 1e-3
tire_step_size = step_size


tend = 1000


render_step_size = 1.0 / 50  


out_dir = "./VIPER"





print( "Copyright (c) 2017 projectchrono.org\n")






viper = veh.VIPER()
viper.SetContactMethod(chrono.ChContactMethod_NSC)
viper.SetChassisFixed(False)
viper.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
viper.SetTireType(veh.TireModelType_TMEASY)
viper.SetTireStepSize(tire_step_size)
viper.SetDriveline8WDType(veh.DrivelineTypeV8_RIGID)
viper.SetDrivelineStepSize(step_size)
viper.SetBrakeType(veh.BrakeType_SIMPLE)
viper.SetBrakeStepSize(step_size)
viper.SetSteeringType(veh.SteeringTypePITMAN_ARM)
viper.SetSteeringStepSize(step_size)
viper.SetEngineType(veh.EngineModelType_SHAFTS)
viper.SetEngineStepSize(step_size)
viper.SetTransmissionType(veh.TransmissionType_AUTOMATIC_SHAFTS)
viper.SetTransmissionStepSize(step_size)
viper.Initialize()

viper.SetChassisVisualizationType(chassis_vis_type)
viper.SetSuspensionVisualizationType(suspension_vis_type)
viper.SetSteeringVisualizationType(steering_vis_type)
viper.SetWheelVisualizationType(wheel_vis_type)

viper.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


minfo = chrono.ChContactMaterialData()
minfo.mu = 0.8
minfo.cr = 0.01
minfo.Y = 2e7
patch_mat = minfo.CreateMaterial(viper.GetSystem(), minfo)
terrain = veh.RigidTerrain(viper.GetSystem())
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 200, 100)
patch.SetColor(chrono.ChColor(1, 1, 1))
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
terrain.Initialize()


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('V8')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 10.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(viper.GetVehicle())


driver = veh.ChInteractiveDriverIRR(vis)


steering_time = 1.0  
throttle_time = 1.0  
braking_time = 0.3   
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()





viper.GetVehicle().EnableRealtime(True)

while vis.Run() :
    time = viper.GetSystem().GetChTime()

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    viper.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    viper.Advance(step_size)
    vis.Advance(step_size)