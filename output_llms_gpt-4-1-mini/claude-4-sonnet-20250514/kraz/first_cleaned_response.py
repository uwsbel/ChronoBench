import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math

def main():
    
    chrono.SetChronoDataPath("../../../data/")
    veh.SetDataPath("../../../data/vehicle/")

    
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))
    
    
    system.SetSolverMaxIterations(150)
    system.SetSolverForceTolerance(1e-10)

    
    terrain = veh.RigidTerrain(system)
    
    
    patch_mat = chrono.ChMaterialSurfaceNSC()
    patch_mat.SetFriction(0.8)
    patch_mat.SetRestitution(0.01)
    patch_mat.SetYoungModulus(2e7)
    patch_mat.SetPoissonRatio(0.3)
    
    
    patch = terrain.AddPatch(patch_mat, 
                           chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT),
                           200.0, 200.0)  
    patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 100, 100)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    
    kraz = veh.Kraz()
    
    
    init_pos = chrono.ChVectorD(0, 0, 1.0)  
    init_rot = chrono.ChQuaternionD(1, 0, 0, 0)  
    
    
    kraz.SetContactMethod(chrono.ChContactMethod_NSC)
    kraz.SetChassisCollisionType(veh.CollisionType_NONE)
    kraz.SetChassisFixed(False)
    kraz.SetInitPosition(chrono.ChCoordsysD(init_pos, init_rot))
    kraz.SetTireType(veh.TireModelType_TMEASY)
    kraz.SetTireStepSize(1e-3)
    kraz.Initialize()

    
    kraz.SetChassisVisualizationType(veh.VisualizationType_MESH)
    kraz.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    kraz.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    kraz.SetWheelVisualizationType(veh.VisualizationType_MESH)
    kraz.SetTireVisualizationType(veh.VisualizationType_MESH)

    
    driver = veh.ChInteractiveDriverIRR()
    
    
    steering_time = 1.0  
    throttle_time = 1.0  
    braking_time = 0.3   
    
    driver.SetSteeringDelta(0.02)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.02)
    
    
    driver.Initialize()

    
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle("Kraz Vehicle Simulation")
    vis.SetWindowSize(1280, 720)
    vis.SetChaseCamera(chrono.ChVectorD(0.0, 0.0, 1.75), 8.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVectorD(-8, 5, 3), chrono.ChVectorD(0, 0, 0))
    vis.AddTypicalLights()
    vis.AttachVehicle(kraz.GetVehicle())

    
    step_size = 1e-3  
    realtime_timer = chrono.ChRealtimeStepTimer()
    time = 0
    
    
    while vis.Run():
        time = kraz.GetSystem().GetChTime()
        
        
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        
        
        driver_inputs = driver.GetInputs()
        
        
        driver.Synchronize(time)
        terrain.Synchronize(time)
        kraz.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)
        
        
        driver.Advance(step_size)
        terrain.Advance(step_size)
        kraz.Advance(step_size)
        vis.Advance(step_size)
        
        
        realtime_timer.Spin(step_size)

    return 0

if __name__ == "__main__":
    main()