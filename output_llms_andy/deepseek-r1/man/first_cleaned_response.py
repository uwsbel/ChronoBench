import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

def main():
    
    
    
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())
    veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')
    
    
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))
    
    
    
    
    
    truck = veh.MAN_10t()
    truck.SetContactMethod(chrono.ChContactMethod_NSC)
    truck.SetChassisFixed(False)
    truck.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1.0), chrono.QUNIT))
    truck.SetTireType(veh.TireModelType_TMEASY)
    truck.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    
    
    truck.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    truck.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    truck.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    truck.SetWheelVisualizationType(veh.VisualizationType_MESH)
    truck.SetTireVisualizationType(veh.VisualizationType_MESH)
    
    
    truck.Initialize()
    
    
    for tire in truck.GetTires():
        tire.Initialize()
        tire.SetVisualization(True)
    
    
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChMaterialSurfaceNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    terrain.SetContactMaterial(patch_mat)
    terrain.SetColor(chrono.ChColor(0.5, 0.5, 0.7))
    terrain.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 200, 200)
    patch = terrain.AddPatch(chrono.ChCoordsysD(), 200, 200)
    patch.SetTextureRepeat(20, 20)
    terrain.Initialize()
    
    
    
    
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('MAN 10t Truck Simulation')
    vis.SetChaseCamera(chrono.ChVectorD(0.0, 0.0, 2.0), 8.0, 0.5)
    vis.Initialize()
    vis.AddLightDirectional(chrono.ChVectorD(0, 30, 50))
    vis.AddSkyBox()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AttachSystem(system)
    vis.AddVehicle(truck.GetVehicle())
    
    
    
    
    driver = veh.ChInteractiveDriverIRR(vis)
    driver.SetSteeringDelta(0.02)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.06)
    driver.Initialize()
    
    
    
    
    truck.GetVehicle().EnableRealtime(True)
    system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
    system.SetSolverMaxIterations(150)
    
    
    
    
    step_size = 0.002
    while vis.Run():
        time = system.GetChTime()
        
        
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        
        
        driver_inputs = driver.GetInputs()
        
        
        driver.Synchronize(time)
        truck.Synchronize(time, driver_inputs, terrain)
        terrain.Synchronize(time)
        vis.Synchronize(time, driver_inputs)
        
        
        truck.Advance(step_size)
        terrain.Advance(step_size)
        vis.Advance(step_size)
        system.DoStepDynamics(step_size)

if __name__ == "__main__":
    main()