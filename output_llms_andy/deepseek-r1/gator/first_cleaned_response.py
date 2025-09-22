import os
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr


chrono.SetChronoDataPath(os.path.join(os.environ.get('CHRONO_DATA_DIR', ''), 'data/'))
veh.SetDataPath(os.path.join(os.environ.get('CHRONO_DATA_DIR', ''), 'vehicle/'))


system = chrono.ChSystemSMC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


gator = veh.Gator(system)
gator.SetContactMethod(chrono.ChContactMethod_SMC)
gator.SetChassisFixed(False)
gator.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0), chrono.QUNIT))
gator.SetTireType(veh.TireModelType_TMEASY)
gator.Initialize()


gator.SetChassisVisualizationType(veh.VisualizationType_MESH)
gator.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
gator.SetSteeringVisualizationType(veh.VisualizationType_MESH)
gator.SetWheelVisualizationType(veh.VisualizationType_MESH)
gator.SetTireVisualizationType(veh.VisualizationType_MESH)


terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChMaterialSurfaceSMC()
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysD(), 100, 100)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
terrain.Initialize()


vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Gator Vehicle Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(6, 3, 6), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()


driver = veh.ChInteractiveDriverIRR(vis)
steering_controller = veh.ChSteeringControllerClosedLoop()
driver.SetSteeringController(steering_controller)
driver.SetGains(0.5, 0, 0)
driver.Initialize()


step_size = 0.01
render_step = 1.0 / 50  
time = 0.0
realtime_timer = chrono.ChRealtimeStepTimer()


while vis.Run():
    time = system.GetChTime()
    
    
    driver.Synchronize(time)
    
    
    gator.Synchronize(time, driver.GetInputs(), terrain)
    
    
    system.DoStepDynamics(step_size)
    
    
    vis.BeginScene(True, True, irr.SColor(255, 140, 161, 192))
    vis.Render()
    vis.Synchronize(time, driver.GetInputs())
    vis.EndScene()
    
    
    realtime_timer.Spin(step_size)

print("Simulation completed.")