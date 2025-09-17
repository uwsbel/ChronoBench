import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import os

def main():
    
    chrono.SetChronoDataPath(os.path.join(os.environ.get('CHRONO_DATA_DIR', ''), ''))
    veh.SetDataPath(os.path.join(os.environ.get('CHRONO_DATA_DIR', ''), 'vehicle/'))

    
    system = chrono.ChSystemSMC()
    system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))

    
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)
    hmmwv.SetChassisFixed(False)
    hmmwv.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1.0), chrono.ChQuaternionD(1, 0, 0, 0)))
    hmmwv.SetTireType(veh.TireModelType_RIGID)
    hmmwv.SetTireStepSize(1e-3)
    hmmwv.Initialize()

    
    hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetSteeringVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

    
    terrain = veh.SCMDeformableTerrain(system)
    terrain.SetPlane(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, -0.5), chrono.QUNIT))
    terrain.Initialize(20.0, 20.0, 0.04)  

    
    soil_params = veh.SoilParametersCallback()
    soil_params.Kphi = 5301e3   
    soil_params.Kc = 102e3      
    soil_params.n = 1.1
    soil_params.cohesion = 1.4e3  
    soil_params.phi = 31.0        
    soil_params.K = 1.2e-2        
    soil_params.E_elastic = 3e6   
    soil_params.damping = 3e4     
    terrain.SetSoilParameters(soil_params)

    
    patch_material = chrono.ChMaterialSurfaceSMC()
    terrain.AddMovingPatch(hmmwv.GetChassisBody(), chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(3, 3, 1))
    terrain.SetPlotType(veh.SCMDeformableTerrain.PLOT_SINKAGE, 0, 0.2)  

    
    vis = irr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle('HMMWV on SCM Terrain')
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVectorD(3, 6, 1.5), chrono.ChVectorD(0, 0, 0))
    vis.AddTypicalLights()

    
    vis.EnableShadows()
    hmmwv.GetVehicle().Visualize(vis)

    
    driver = veh.ChIrrGuiDriver(vis)
    driver.SetSteeringDelta(0.02)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.06)
    driver.Initialize()

    
    step_size = 0.002
    render_step = 1.0 / 50  

    realtime_timer = chrono.ChRealtimeStepTimer()
    time = 0.0
    frame_number = 0

    while vis.Run():
        time = system.GetChTime()

        
        driver_inputs = driver.GetInputs()
        hmmwv.GetVehicle().Synchronize(time, driver_inputs, terrain)
        terrain.Synchronize(time)
        vis.Synchronize(time, driver_inputs)

        
        driver.Advance(step_size)
        terrain.Advance(step_size)
        hmmwv.GetVehicle().Advance(step_size)
        system.DoStepDynamics(step_size)

        
        if frame_number % int(render_step / step_size) == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        realtime_timer.Spin(step_size)
        frame_number += 1

if __name__ == "__main__":
    main()