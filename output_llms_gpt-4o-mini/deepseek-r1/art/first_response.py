import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math

# 1. Initialize PyChrono environment and core components
def main():
    # Set path to Chrono data directory
    chrono.SetChronoDataPath("path/to/chrono/data/")
    
    # Create the physical system
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
    
    # 2. Create and initialize the ARTcar vehicle
    initial_pos = chrono.ChVectorD(0, 1, 0)  # Start 1m above ground
    initial_rot = chrono.Q_ROTATE_Y_TO_Z  # Default vehicle orientation
    contact_method = chrono.ChContactMethod_NSC
    vehicle = veh.ARTVehicle(system, contact_method)
    vehicle.Initialize(chrono.ChCoordsysD(initial_pos, initial_rot))
    vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
    
    # 3. Create and configure the rigid terrain
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChMaterialSurfaceNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch = terrain.AddPatch(patch_mat, 
                            chrono.ChVectorD(0, 0, 0), 
                            chrono.ChVectorD(0, 1, 0), 
                            100, 100)  # 100x100 meter patch
    patch.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 200, 200)
    terrain.Initialize()
    
    # 4. Set up Irrlicht visualization
    vis = irr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("ARTcar Simulation")
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVectorD(-8, 2, 0), chrono.ChVectorD(0, 0, 0))
    vis.AddTypicalLights()
    
    # 5. Create interactive driver system
    driver = veh.ChIrrGuiDriver(vis)
    driver.SetThrottleDelta(1.0/50)
    driver.SetSteeringDelta(1.0/50)
    driver.SetBrakingDelta(1.0/50)
    driver.Initialize()
    
    # 6. Simulation loop parameters
    step_size = 0.02  # 50 FPS (1/50 = 0.02)
    realtime_timer = chrono.ChRealtimeStepTimer()
    
    while vis.Run():
        # Update simulation time
        time = system.GetChTime()
        
        # Update driver inputs
        driver_inputs = driver.GetInputs()
        vehicle.Synchronize(time, driver_inputs, terrain)
        terrain.Synchronize(time)
        
        # Advance simulation
        system.DoStepDynamics(step_size)
        
        # Update visualization
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        
        # Display vehicle speed
        speed = vehicle.GetChassisBody().GetPos_dt().Length() * 3.6  # Convert m/s to km/h
        vis.GetGUIEnvironment().addStaticText(
            f"Time: {time:.2f}s | Speed: {speed:.2f} km/h", 
            irr.recti(10, 10, 400, 40))
        
        # Maintain real-time simulation
        realtime_timer.Spin(step_size)

if __name__ == "__main__":
    main()