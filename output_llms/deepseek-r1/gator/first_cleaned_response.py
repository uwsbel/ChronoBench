import os
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr




def main():
    
    chrono.SetChronoDataPath(os.path.join(os.environ.get('CHRONO_DATA_DIR', ''), 'data/'))
    veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')
    
    
    contact_method = chrono.ChContactMethod_SMC
    step_size = 0.005
    render_fps = 50
    render_step_size = 1.0 / render_fps
    
    
    system = chrono.ChSystem(contact_method)
    system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))
    system.SetSolverMaxIterations(150)
    system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
    
    
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChMaterialSurfaceSMC()
    patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysD(), 200, 100)
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()
    
    
    init_loc = chrono.ChVectorD(0, 0, 0.5)
    init_rot = chrono.ChQuaternionD(1, 0, 0, 0)
    gator = veh.Gator(system)
    gator.SetInitPosition(chrono.ChCoordsysD(init_loc, init_rot))
    gator.SetTireType(veh.TireModelType_TMEASY)
    gator.Initialize()
    
    
    gator.SetChassisVisualizationType(veh.VisualizationType_MESH)
    gator.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
    gator.SetSteeringVisualizationType(veh.VisualizationType_MESH)
    gator.SetWheelVisualizationType(veh.VisualizationType_MESH)
    gator.SetTireVisualizationType(veh.VisualizationType_MESH)
    
    
    vis = irr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle('Gator Vehicle Simulation')
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVectorD(6, 6, 1.5), chrono.ChVectorD(0, 0, 0.5))
    vis.AddTypicalLights()
    vis.AddLightWithShadow(chrono.ChVectorD(10, 10, 10), chrono.ChVectorD(0, 0, 0), 50, 5, 50, 50, 512)
    
    
    driver = veh.ChIrrGuiDriver(vis)
    driver.SetSteeringDelta(0.02)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.06)
    driver.Initialize()
    
    
    realtime_timer = chrono.ChRealtimeStepTimer()
    step_number = 0
    
    while vis.Run():
        time = system.GetChTime()
        
        
        vis.BeginScene(True, True, irr.SColor(255, 140, 161, 192))
        vis.Render()
        vis.EndScene()
        
        
        driver_inputs = driver.GetInputs()
        
        
        gator.Synchronize(time, driver_inputs, terrain)
        terrain.Synchronize(time)
        driver.Synchronize(time)
        
        
        system.DoStepDynamics(step_size)
        
        
        vis.Synchronize(time, driver_inputs)
        
        
        step_number += 1
        
        
        realtime_timer.Spin(step_size)




if __name__ == "__main__":
    main()