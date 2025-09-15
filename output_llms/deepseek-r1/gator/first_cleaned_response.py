import os
import math
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr






chrono.SetChronoDataPath(os.path.join(os.environ['CHRONO_DATA_DIR'], ''))


system = chrono.ChSystemSMC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))






init_loc = chrono.ChVectorD(0, 0.5, 0)
init_rot = chrono.ChQuaternionD(1, 0, 0, 0)
tire_type = veh.TMeasyTire
step_size = 0.002


gator = veh.Gator()
gator.SetContactMethod(chrono.ChContactMethod_SMC)
gator.SetInitPosition(chrono.ChCoordsysD(init_loc, init_rot))
gator.SetTireType(tire_type)
gator.Initialize()


gator.SetChassisVisualizationType(veh.VisualizationType_MESH)
gator.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
gator.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
gator.SetWheelVisualizationType(veh.VisualizationType_MESH)






terrain_length = 200.0
terrain_width = 200.0
terrain_thickness = 0.2


terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChMaterialSurfaceSMC()
patch = terrain.AddPatch(patch_mat, 
                        chrono.ChVectorD(0, 0, 0), 
                        chrono.ChVectorD(0, 0, 1),
                        terrain_length, terrain_width)
patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()






vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Gator Vehicle Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(6, 6, 1.5))
vis.AddTypicalLights()


gator.AddVisualizationAssets(irr.VISUALIZE_MESH)






driver = veh.ChIrrGuiDriver(vis)


driver.Initialize()
driver.SetDesiredSpeed(5.0)  






realtime_step = 0.02  
render_step = 1.0 / 50.0  
time = 0.0

while vis.Run():
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    
    time = system.GetChTime()
    
    
    driver_inputs = driver.GetInputs()
    gator.DriverInputs = driver_inputs
    
    
    gator.Synchronize(time, driver_inputs, terrain)
    terrain.Synchronize(time)
    
    
    system.DoStepDynamics(step_size)
    
    
    chrono.ChRealtimeStepTimer.Sleep(realtime_step)
    
    
    if math.fmod(time, 1.0) < step_size:
        print(f"Time: {time:.2f}s Speed: {gator.GetVehicle().GetSpeed():.2f} m/s")