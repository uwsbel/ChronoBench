import os
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

def main():
    
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
    system.SetSolverMaxIterations(100)
    
    
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
    hmmwv.SetChassisFixed(False)
    hmmwv.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0), chrono.QUNIT))
    hmmwv.SetTireType(veh.TireModelType_TMEASY)
    hmmwv.Initialize()
    
    
    hmmwv.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)
    
    
    terrain = veh.SCMDeformableTerrain(system)
    terrain.SetPlane(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngX(-0.5)))
    
    
    soil_params = veh.SoilParameters()
    soil_params.Kphi = 5301e3
    soil_params.Kc = 102e3
    soil_params.n = 1.1
    soil_params.cohesion = 0
    soil_params.phi = 31 * chrono.CH_DEG_TO_RAD
    soil_params.K = 1.8e-2
    soil_params.E_elastic = 3e6
    soil_params.damping = 0.01
    
    
    terrain.SetSoilParameters(soil_params)
    terrain.Initialize(20, 10, 0.04)  
    
    
    driver = veh.ChPathFollowerDriver(hmmwv.GetVehicle())
    path = chrono.ChBezierCurve()
    path.AddPoint(chrono.ChVectorD(-10, 0, 0), chrono.ChVectorD(10, 0, 0))
    driver.GetSteeringController().SetLookAheadDistance(5.0)
    driver.GetSteeringController().SetGains(0.5, 0, 0)
    driver.GetSpeedController().SetGains(0.4, 0, 0)
    driver.SetPath(path)
    
    
    vis = irr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("HMMWV on SCM Terrain")
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVectorD(5, 2, 5), chrono.ChVectorD(0, 0, 0))
    vis.AddTypicalLights()
    vis.AddLightWithShadow(chrono.ChVectorD(10, 10, 10), chrono.ChVectorD(0, 0, 0), 20, 5, 20, 40, 512)
    vis.EnableShadows()
    
    
    step_size = 0.002
    realtime_timer = chrono.ChRealtimeStepTimer()
    
    while vis.Run():
        time = system.GetChTime()
        
        
        driver.Synchronize(time)
        
        
        hmmwv.Synchronize(time, driver.GetInputs(), terrain)
        terrain.Synchronize(time)
        
        
        system.DoStepDynamics(step_size)
        
        
        vis.BeginScene()
        vis.Render()
        vis.RenderFrame(chrono.ChFrameD(), 2)  
        vis.EndScene()
        
        
        realtime_timer.Spin(step_size)

if __name__ == "__main__":
    main()