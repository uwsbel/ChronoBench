import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

def main():
    
    
    
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())
    veh.SetDataPath(veh.GetDataPath() + 'vehicle/')
    
    
    system = chrono.ChSystemSMC()
    system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))
    
    
    
    
    terrain = veh.SCMDeformableTerrain(system)
    terrain.SetPlane(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, -0.5), chrono.Q_from_AngX(-math.pi/2)))
    
    
    soil_params = veh.SoilParametersCallback()
    soil_params.m_Kn = 2e6   
    soil_params.m_Kt = 1e6   
    soil_params.m_n = 1.1    
    soil_params.m_moist_cohesion = 0.0
    soil_params.m_moist_friction = 30.0
    soil_params.m_dry_cohesion = 0.0
    soil_params.m_dry_friction = 35.0
    terrain.SetSoilParameters(soil_params)
    
    
    terrain.SetPlotType(veh.SCMDeformableTerrain.PLOT_SINKAGE, 0, 0.15)
    terrain.SetColorTableMin(0.0)
    terrain.SetColorTableMax(0.15)
    
    
    patch_size = 5.0
    terrain.Initialize(0.0, 0.0, patch_size, patch_size, 0.04)
    
    
    
    
    
    hmmwv = veh.HMMWV_Full(system)
    hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)
    hmmwv.SetChassisFixed(False)
    hmmwv.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1.0), chrono.Q_from_AngZ(0)))
    hmmwv.SetTireType(veh.TireModelType_RIGID)
    hmmwv.Initialize()
    
    
    hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetSteeringVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)
    
    
    
    
    vis = irr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle('HMMWV on SCM Terrain')
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVectorD(8, 8, 1.5), chrono.ChVectorD(0, 0, 0.5))
    vis.AddTypicalLights()
    
    
    
    
    driver = veh.ChInteractiveDriverIRR(vis)
    driver.SetSteeringDelta(0.02)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.06)
    driver.Initialize()
    
    
    
    
    step_size = 0.002
    render_step = 1.0 / 50  
    
    realtime_timer = chrono.ChRealtimeStepTimer()
    time = 0
    frame_number = 0
    
    while vis.Run():
        time = system.GetChTime()
        
        
        chassis_pos = hmmwv.GetVehicle().GetChassisBody().GetPos()
        terrain.SetPatchCenter(chassis_pos.x, chassis_pos.y)
        
        
        if frame_number % int(render_step / step_size) == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
        
        
        driver_inputs = driver.GetInputs()
        
        
        driver.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        terrain.Synchronize(time)
        vis.Synchronize(time, driver_inputs)
        
        
        hmmwv.Advance(step_size)
        terrain.Advance(step_size)
        system.DoStepDynamics(step_size)
        
        
        realtime_timer.Spin(step_size)
        frame_number += 1

if __name__ == "__main__":
    main()