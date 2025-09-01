import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr


initLoc = chrono.ChVector3d(0, 0, 1.0)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


step_size = 2e-3


render_step_size = 1.0 / 50


chassis_collision_type = veh.CollisionType_NONE


contact_method = chrono.ChContactMethod_NSC


tire_model = veh.TireModelType_TMEASY


my_hmmwv = veh.HMMWV_Full()
my_hmmwv.SetContactMethod(contact_method)
my_hmmwv.SetChassisCollisionType(chassis_collision_type)
my_hmmwv.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
my_hmmwv.SetTireType(tire_model)
my_hmmwv.SetTireStepSize(step_size)
my_hmmwv.Initialize()


vis_type = veh.VisualizationType_PRIMITIVES


my_hmmwv.SetChassisVisualizationType(vis_type)
my_hmmwv.SetSuspensionVisualizationType(vis_type)
my_hmmwv.SetSteeringVisualizationType(vis_type)
my_hmmwv.SetWheelVisualizationType(vis_type)
my_hmmwv.SetTireVisualizationType(vis_type)


terrain = veh.RigidTerrain(my_hmmwv.GetSystem())
patch_mat = chrono.ChMaterialSurfaceNSC() if contact_method == chrono.ChContactMethod_NSC else chrono.ChMaterialSurfaceSMC()
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 100, 100)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(my_hmmwv.GetSystem())
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('HMMWV Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(10, 10, -5))
vis.AddTypicalLights()


driver = veh.ChIrrGuiDriver(vis)
driver.SetSteeringDelta(0.02)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.06)
my_hmmwv.SetDriver(driver)


realtime_timer = chrono.ChRealtimeStepTimer()
while vis.Run():
    time = my_hmmwv.GetSystem().GetChTime()

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    driver_inputs = driver.GetInputs()
    my_hmmwv.Synchronize(time, driver_inputs, terrain)
    my_hmmwv.Advance(step_size)

    
    realtime_timer.Spin(step_size)