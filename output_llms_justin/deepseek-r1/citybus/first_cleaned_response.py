import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh

def main():
    
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())
    system = chrono.ChSystemNSC()
    system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
    system.SetSolverMaxIterations(150)
    system.SetMaxPenetrationRecoverySpeed(4.0)

    
    vehicle = veh.CityBus()
    vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
    vehicle.SetChassisFixed(False)
    vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0), chrono.QUNIT))
    vehicle.SetTireType(veh.TireModelType_TMEASY)
    vehicle.SetTireStepSize(0.001)
    vehicle.Initialize()

    
    vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)

    
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChMaterialSurfaceNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch = terrain.AddPatch(patch_mat, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 1), 100, 100)
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    terrain.Initialize()

    
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle('CityBus Simulation')
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVectorD(6, 2, 6))
    vis.AddTypicalLights()
    vis.AddLightWithShadow(chrono.ChVectorD(-10, 20, -10), chrono.ChVectorD(0,0,0), 50, 10, 40, 60, 512)

    
    driver = veh.ChIrrGuiDriver(vis)
    driver.SetSteeringDelta(0.02)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.06)
    driver.Initialize()

    
    vehicle.GetVehicle().EnableRealtime(True)
    vehicle.GetVehicle().SetChassisVisualization(True)

    
    step_size = 0.002
    frame_interval = 0.02  
    time = 0
    frame_number = 0

    while vis.Run():
        time = system.GetChTime()
        
        
        driver_inputs = driver.GetInputs()
        vehicle.GetVehicle().Synchronize(time, driver_inputs, terrain)
        terrain.Synchronize(time)
        
        
        driver.Advance(step_size)
        terrain.Advance(step_size)
        vehicle.Advance(step_size)
        vis.Advance(step_size)
        
        
        if time >= frame_number * frame_interval:
            vis.BeginScene()
            vis.Render()
            vis.RenderFrame(vehicle.GetVehicle().GetChassisBody())
            vis.EndScene()
            frame_number += 1
        
        system.DoStepDynamics(step_size)

if __name__ == "__main__":
    main()