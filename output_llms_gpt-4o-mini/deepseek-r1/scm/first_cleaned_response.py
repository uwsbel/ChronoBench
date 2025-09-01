import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math


FRAME_RATE = 50  
STEP_SIZE = 0.01  

def main():
    
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))
    system.SetSolverMaxIterations(100)
    system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)

    
    vehicle = veh.HMMWV_Full()
    vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
    vehicle.SetChassisFixed(False)
    vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.5), 
                                             chrono.Q_from_AngY(math.pi/2)))
    vehicle.SetTireType(veh.TireModelType_RIGID)
    vehicle.Initialize()

    
    vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetSteeringVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)

    
    terrain = veh.SCMDeformableTerrain(system)
    terrain.SetSoilParameters(
        Kphi = 4.0680e6,    
        Kc = 2.0680e5,      
        n = 1.1,            
        K = 2.52e-2,        
        coh = 3.45e3,       
        phi_deg = 31.4,     
        E_elastic = 2e8,    
        damping = 3e4       
    )
    
    
    terrain.SetPlotType(veh.SCMDeformableTerrain.PLOT_SINKAGE, 0, 0.2)
    terrain.Initialize(chrono.ChVectorD(-5, -3, 0), 
                      chrono.ChVectorD(10, 6, 0.1), 
                      0.04)  
    
    
    terrain.AddMovingPatch(vehicle.GetChassisBody(), 
                          chrono.ChVectorD(0, 0, 0), 
                          chrono.ChVectorD(5, 3, 1))

    
    vis = irr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle('HMMWV on SCM Terrain')
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVectorD(3, 3, 1.5), chrono.ChVectorD(0, 0, 0))
    vis.AddTypicalLights()

    
    driver = veh.ChIrrGuiDriver(vis)
    driver.SetSteeringDelta(0.02)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.06)
    driver.Initialize()

    
    time = 0.0
    frame_number = 0
    timer = chrono.ChTimer()
    timer.start()

    while vis.Run():
        time = system.GetChTime()
        
        
        driver_inputs = driver.GetInputs()
        vehicle.Synchronize(time, driver_inputs, terrain)
        terrain.Synchronize(time)
        vis.Synchronize(time, driver_inputs)
        
        
        driver.Advance(STEP_SIZE)
        terrain.Advance(STEP_SIZE)
        vehicle.Advance(STEP_SIZE)
        system.DoStepDynamics(STEP_SIZE)
        
        
        if frame_number % (1 / (FRAME_RATE * STEP_SIZE)) == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
        
        frame_number += 1
        timer.spin(STEP_SIZE)

    timer.stop()
    return 0

if __name__ == '__main__':
    main()