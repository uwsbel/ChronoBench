#!/usr/bin/env python3
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

def main():
    # ==========================================================================
    # 1. Initialize PyChrono environment and core components
    # ==========================================================================
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())
    veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')
    
    # Create the physical system
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))
    
    # ==========================================================================
    # 2. Create the vehicle and terrain systems
    # ==========================================================================
    # Initialize the MAN 10t truck
    truck = veh.MAN_10t()
    truck.SetContactMethod(chrono.ChContactMethod_NSC)
    truck.SetChassisFixed(False)
    truck.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1.0), chrono.QUNIT))
    truck.SetTireType(veh.TireModelType_TMEASY)
    truck.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    
    # Set visualization modes
    truck.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    truck.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    truck.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    truck.SetWheelVisualizationType(veh.VisualizationType_MESH)
    truck.SetTireVisualizationType(veh.VisualizationType_MESH)
    
    # Initialize the vehicle system
    truck.Initialize()
    
    # Set tire parameters
    for tire in truck.GetTires():
        tire.Initialize()
        tire.SetVisualization(True)
    
    # Create rigid terrain
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
    
    # ==========================================================================
    # 3. Set up visualization system
    # ==========================================================================
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('MAN 10t Truck Simulation')
    vis.SetChaseCamera(chrono.ChVectorD(0.0, 0.0, 2.0), 8.0, 0.5)
    vis.Initialize()
    vis.AddLightDirectional(chrono.ChVectorD(0, 30, 50))
    vis.AddSkyBox()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AttachSystem(system)
    vis.AddVehicle(truck.GetVehicle())
    
    # ==========================================================================
    # 4. Configure driver controls
    # ==========================================================================
    driver = veh.ChInteractiveDriverIRR(vis)
    driver.SetSteeringDelta(0.02)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.06)
    driver.Initialize()
    
    # ==========================================================================
    # 5. Configure simulation parameters
    # ==========================================================================
    truck.GetVehicle().EnableRealtime(True)
    system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
    system.SetSolverMaxIterations(150)
    
    # ==========================================================================
    # 6. Simulation loop
    # ==========================================================================
    step_size = 0.002
    while vis.Run():
        time = system.GetChTime()
        
        # Update visualization
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        
        # Get driver inputs
        driver_inputs = driver.GetInputs()
        
        # Update systems
        driver.Synchronize(time)
        truck.Synchronize(time, driver_inputs, terrain)
        terrain.Synchronize(time)
        vis.Synchronize(time, driver_inputs)
        
        # Advance simulation
        truck.Advance(step_size)
        terrain.Advance(step_size)
        vis.Advance(step_size)
        system.DoStepDynamics(step_size)

if __name__ == "__main__":
    main()