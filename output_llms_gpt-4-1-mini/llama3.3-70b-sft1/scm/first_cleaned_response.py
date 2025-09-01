import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens
import math
import os








veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVector3d(0, 0, 1.6)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


chassis_vis_type = veh.VisualizationType_MESH
suspension_vis_type = veh.VisualizationType_PRIMITIVES
steering_vis_type = veh.VisualizationType_PRIMITIVES
wheel_vis_type = veh.VisualizationType_MESH
tire_vis_type = veh.VisualizationType_MESH


trackPoint = chrono.ChVector3d(0.0, 0.0, 1.75)


step_size = 2e-3
tire_step_size = step_size


tend = 1000


render_step_size = 1 / 50  


out_dir = "./HMMWV"





print( "Copyright (c) 2017 projectchrono.org\n")






hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
hmmwv.SetTireType(veh.TireModelType_RIGID)
hmmwv.SetSoilType(veh.SoilType_SCM)
hmmwv.SetChassisVisualizationType(chassis_vis_type)
hmmwv.SetSuspensionVisualizationType(suspension_vis_type)
hmmwv.SetSteeringVisualizationType(steering_vis_type)
hmmwv.SetWheelVisualizationType(wheel_vis_type)
hmmwv.SetTireVisualizationType(tire_vis_type)
hmmwv.SetChassisRearVisualizationType(chassis_vis_type)
hmmwv.SetChassisFrontVisualizationType(chassis_vis_type)
hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
hmmwv.SetTireStepSize(tire_step_size)
hmmwv.Initialize()

hmmwv.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_NONE)
hmmwv.SetTireVisualizationType(veh.VisualizationType_NONE)
hmmwv.SetChassisRearVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetChassisFrontVisualizationType(veh.VisualizationType_PRIMITIVES)

hmmwv.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)



terrain = veh.SCMTerrain(hmmwv.GetSystem())
patch_mat = chrono.ChContactMaterialSMC()
patch_mat.SetKphi(0)
patch_mat.SetKc(0)
patch_mat.SetN(1.1)
patch = terrain.AddPatch(patch_mat, 
                         chrono.CSYSNORM, 
                         200, 100, 
                         0.04)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetNormalTexture(veh.GetDataFile("terrain/textures/tile4_N.jpg"), 200, 200)
terrain.Initialize()


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Deformable Soil Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(hmmwv.GetVehicle())


driver = veh.ChInteractiveDriverIRR(vis)


steering_time = 1.0  
throttle_time = 1.0  
braking_time = 0.3   
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()





hmmwv.GetVehicle().EnableRealtime(True)

while vis.Run() :
    time = hmmwv.GetSystem().GetChTime()

    
    if (time >= tend):
        break

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    hmmwv.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    hmmwv.Advance(step_size)
    vis.Advance(step_size)