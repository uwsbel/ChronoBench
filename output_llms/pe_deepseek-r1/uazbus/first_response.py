import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

def main():
    # ==========================================================================
    # 1. Initialize PyChrono environment
    # ==========================================================================
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())
    
    # Create system with SMC contact method
    system = chrono.ChSystemSMC()
    system.SetGravity(chrono.ChVector3d(0, 0, -9.81))
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    # ==========================================================================
    # 2. Create terrain
    # ==========================================================================
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChContactMaterialSMC()
    patch_mat.SetFriction(0.9)      # High friction for vehicle traction
    patch_mat.SetRestitution(0.01)  # Low restitution for hard surfaces
    patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 100, 50)
    patch.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 100, 50)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    # ==========================================================================
    # 3. Create and initialize UAZBUS vehicle
    # ==========================================================================
    uaz = veh.UAZBUS()
    uaz.SetContactMethod(chrono.ChContactMethod_SMC)
    uaz.SetChassisFixed(False)
    uaz.SetInitPosition(chrono.ChCoordsysd(
        chrono.ChVector3d(0, 0, 0.5),  # Initial position (0.5m above ground)
        chrono.QUNIT                  # Initial rotation (no rotation)
    ))
    uaz.Initialize()

    # Configure vehicle subsystems
    uaz.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    uaz.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    uaz.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    uaz.SetWheelVisualizationType(veh.VisualizationType_MESH)

    # ==========================================================================
    # 4. Create driver system
    # ==========================================================================
    driver = veh.ChInteractiveDriverIRR(uaz.GetVehicle())
    
    # Configure driver inputs
    driver.SetSteeringDelta(0.02)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.06)
    driver.Initialize()

    # ==========================================================================
    # 5. Setup visualization system
    # ==========================================================================
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("UAZBUS Simulation")
    vis.Initialize()
    
    # Configure camera position and lighting
    vis.AddCamera(chrono.ChVector3d(3, 3, 1.5), chrono.ChVector3d(0, 0, 0.5))
    vis.AddTypicalLights()
    vis.AddSkyBox()
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    
    # Attach vehicle-specific visualization
    uaz.GetVehicle().AddVisualSystem(vis)

    # ==========================================================================
    # 6. Simulation loop
    # ==========================================================================
    step_size = 0.002
    realtime_timer = chrono.ChRealtimeStepTimer()

    while vis.Run():
        time = system.GetChTime()
        
        # Render scene
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        
        # Update driver inputs
        driver.Synchronize(time)
        
        # Advance simulation
        uaz.Synchronize(time, driver.GetInputs(), terrain)
        terrain.Synchronize(time)
        system.DoStepDynamics(step_size)
        
        # Maintain real-time synchronization
        realtime_timer.Spin(step_size)

if __name__ == "__main__":
    main()