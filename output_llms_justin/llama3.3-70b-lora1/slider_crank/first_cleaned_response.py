import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math







change_camera_angles = True


step_size = 5e-3


left_width = 3.5
right_width = 3.0




hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(contact_method)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
hmmwv.SetEngineType(engine_model)
hmmwv.SetTransmissionType(transmission_model)
hmmwv.SetDriveType(drive_type)
hmmwv.SetSteeringType(steering_type)
hmmwv.SetTireType(tire_model)
hmmwv.SetTireStepSize(tire_step_size)
hmmwv.Initialize()

hmmwv.SetChassisVisualizationType(chassis_vis_type)
hmmwv.SetSuspensionVisualizationType(suspension_vis_type)
hmmwv.SetSteeringVisualizationType(steering_vis_type)
hmmwv.SetWheelVisualizationType(wheel_vis_type)
hmmwv.SetTireVisualizationType(tire_vis_type)

hmmwv.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


minfo = chrono.ChContactMaterialData()
minfo.mu = 0.8
minfo.cr = 0.01
minfo.Y = 1e7
patch_mat = minfo.CreateMaterial(hmmwv.GetSystem(), minfo)

terrain = veh.RigidTerrain(hmmwv.GetSystem())
patch = terrain.AddPatch(patch_mat, 
                         chrono.ChCoordsysd(chrono.ChVector3d(-10, 0, 0), chrono.QUNIT), 
                         200, 20)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


steer_time = 0.5
target_speed = 12

driver = veh.ChPathFollowerDriver(steer_time, target_speed, veh.GetDataFile("path/hmmwv.json"))

driver.Initialize(terrain.GetReferenceFrame())


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV-4WD')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(track_point, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(hmmwv.GetVehicle())


env = vis.GetIRRlichtEnvironment()
sphere1 = irr.SphereSceneNode(0.1)
sphere2 = irr.SphereSceneNode(0.1)
mmmm = irr.ChVisualMaterial(irr.CSMC_AOMAP, 1.0, 1.0, 1.0, chrono.ChColor2(1, 1, 1), chrono.ChColor2(0, 0, 0))
sph_mat = irr.ChIrrNodeMaterial(1.0, 1.0, 1.0, 10.0, 0.9, 0.9)
sph_mat.SetDiffuseColor(chrono.ChColor2(0.0, 0.0, 0.0))
mmmm.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"))
sphere1.SetMaterial(mmmm)
sphere2.SetMaterial(sph_mat)


my_sync = False
my_sync2 = False





hmmwv.GetVehicle().EnableRealtime(True)

while vis.Run() :
    time = hmmwv.GetSystem().GetChTime()

    
    if (time >= t_end):
        break

    
    update_drivers(time, driver, hmmwv.GetSteering(), hmmwv.GetThrottle(), hmmwv.GetBraking(), vis)
    driver_inputs = driver.GetInputs()
    hmmwv.SetDriverInputs(driver_inputs)

    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    hmmwv.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    hmmwv.Advance(step_size)
    vis.Advance(step_size)