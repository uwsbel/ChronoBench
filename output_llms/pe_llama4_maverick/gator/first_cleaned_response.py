import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import numpy as np






out_dir = chrono.GetChronoOutputPath() + "GATOR"


contact_method = chrono.ChContactMethod_NSC


time_step = 1e-3


terrainLength = 100.0  
terrainWidth = 100.0  


initLoc = chrono.ChVector3d(0, 0, 1.0)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)  


chassis_vis_type = veh.VisualizationType_MESH
suspension_vis_type = veh.VisualizationType_MESH
steering_vis_type = veh.VisualizationType_MESH
wheel_vis_type = veh.VisualizationType_MESH


tire_model = veh.TireModelType_TMEASY






gator = veh.Gator()
gator.SetContactMethod(contact_method)
gator.SetChassisCollisionType(veh.CollisionType_NONE)
gator.SetChassisVisType(chassis_vis_type)
gator.SetSuspensionVisType(suspension_vis_type)
gator.SetSteeringVisType(steering_vis_type)
gator.SetWheelVisType(wheel_vis_type)
gator.SetTireType(tire_model)
gator.Initialize(chrono.ChCoordsysd(initLoc, initRot))
gator.SetTireStepSize(time_step)


terrain = veh.RigidTerrain(gator.GetSystem())
patch_mat = chrono.ChMaterialSurfaceNSC() if contact_method == chrono.ChContactMethod_NSC else chrono.ChMaterialSurfaceSMC()
patch_mat.SetFriction(0.9)
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), terrainLength, terrainWidth)
patch.SetTexture(chrono.GetChronoDataFile("textures/grass.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(gator.GetSystem())
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('Gator Vehicle Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, -10, 2))
vis.AddTypicalLights()


driver = veh.ChIrrGuiDriver(vis)


steering_time = 1.0  
throttle_time = 1.0  
braking_time = 0.3  
driver.SetSteeringDelta(driver.GetSteeringMax() / steering_time * time_step)
driver.SetThrottleDelta(driver.GetThrottleMax() / throttle_time * time_step)
driver.SetBrakingDelta(driver.GetBrakingMax() / braking_time * time_step)


while vis.Run():
    time = gator.GetSystem().GetChTime()

    
    driver_inputs = driver.GetInputs()
    gator.SetDriverInputs(driver_inputs)

    
    driver.Synchronize(time)
    gator.Synchronize(time, driver_inputs, terrain)
    terrain.Synchronize(time)

    
    driver.Advance(time_step)
    gator.Advance(time_step)
    terrain.Advance(time_step)

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    vis.WaitForEndScene(20)


vis.GetDevice().closeDevice()