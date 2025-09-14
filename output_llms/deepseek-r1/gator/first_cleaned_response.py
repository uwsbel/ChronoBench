import os
import math
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr


chrono.SetChronoDataPath(os.path.join(os.environ['CHRONO_DATA_DIR'], 'data/'))
system = chrono.ChSystemSMC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))


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
patch = terrain.AddPatch(patch_mat, 
                       chrono.ChVectorD(0, 0, 0), 
                       chrono.ChVectorD(0, 0, 1), 
                       100.0, 100.0)
patch.SetTexture(veh.GetDataFile("terrain/textures/terrain_texture.jpg"), 100, 100)
terrain.Initialize()


vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Gator Vehicle Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(3, 3, 1.5))
vis.AddTypicalLights()


driver = veh.ChIrrGuiDriver(vis)
steering_axis = chrono.ChVectorD(0, 0, 0.6)
driver.SetSteeringAxis(steering_axis)
driver.Initialize()


step_size = 0.02
render_step = 1.0 / 50  


time = 0.0
while vis.Run():
    time = system.GetChTime()
    
    
    vis.BeginScene()
    vis.DrawAll()
    driver.Synchronize(time)
    
    
    gator.Synchronize(time, driver.GetInputs(), terrain)
    terrain.Synchronize(time)
    system.DoStepDynamics(step_size)
    
    
    vis.Synchronize(time, driver.GetInputs())
    vis.EndScene()
    
    
    chrono.ChRealtimeStep(render_step)

print("Simulation completed.")