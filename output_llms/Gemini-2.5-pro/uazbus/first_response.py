import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import os

def main():
    # 0. Set path to Chrono data directory
    chrono.SetChronoDataPath(os.environ.get('CHRONO_DATA_DIR', '../../../chrono_data/'))
    veh.SetDataPath(os.path.join(chrono.GetChronoDataPath(), 'vehicle/'))

    # 1. Initialize the PyChrono environment and core components
    # Create a Chrono system
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
    system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
    system.SetSolverMaxIterations(150)
    system.SetMaxPenetrationRecoverySpeed(4.0)

    # 2. Add the required physical systems and objects as specified
    # Create the UAZBUS vehicle
    uaz = veh.UAZBUS()
    uaz.SetContactMethod(chrono.ChMaterialSurface.NSC) # Non-smooth contact
    uaz.SetChassisFixed(False) # Chassis is not fixed to ground

    # Set initial position and orientation of the vehicle
    # Position: x=0, y=0.5 (slightly above ground), z=0
    # Orientation: No rotation (Quaternion: 1,0,0,0)
    initial_pos = chrono.ChVectorD(0, 0.7, 0)
    initial_rot = chrono.Q_from_AngZ(0) # No rotation around Z
    uaz.SetInitPosition(chrono.ChCoordsysD(initial_pos, initial_rot))

    # Set powertrain, driveline, and tire types
    uaz.SetPowertrainType(veh.PowertrainModelType_SHAFTS)
    uaz.SetDriveType(veh.DrivelineTypeWV_AWD) # All-Wheel Drive
    uaz.SetTireType(veh.TireModelType_TMEASY) # TMeasy tires suitable for rigid terrain

    # Initialize the vehicle system
    uaz.Initialize()

    # Set visualization type for vehicle components
    uaz.SetChassisVisualizationType(veh.VisualizationType_MESH)
    uaz.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    uaz.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    uaz.SetWheelVisualizationType(veh.VisualizationType_MESH) # Use MESH for wheels
    uaz.SetTireVisualizationType(veh.VisualizationType_MESH) # Use MESH for tires


    # Create the rigid terrain
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChMaterialSurfaceNSC()
    patch_mat.SetFriction(0.9)       # Defined friction
    patch_mat.SetRestitution(0.01)   # Defined restitution
    
    # Add a large flat patch of terrain
    # Parameters: material, center_point_normal_csys, length_x, width_y
    # Using CSYSNORM means the normal is along Z, so the plane is XY.
    # For a horizontal terrain, we want the normal along Y.
    # Let's create a box as ground for simplicity and clear visualization
    ground_body = chrono.ChBodyEasyBox(200, 1, 200, 1000, True, True, patch_mat)
    ground_body.SetPos(chrono.ChVectorD(0, -0.5, 0)) # Center of the box, making top surface at y=0
    ground_body.SetBodyFixed(True)
    system.Add(ground_body)
    
    # Add texture to the ground for better visualization
    texture = chrono.ChTexture()
    texture.SetTextureFilename(chrono.GetChronoDataPath() + "textures/concrete.jpg")
    ground_body.GetAssets().push_back(texture)
    
    # Initialize the terrain (even if it's just a body, this can be useful for vehicle interaction)
    # For custom body terrain, we effectively do this by adding the body to the system.
    # If using veh.RigidTerrain patches:
    # patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 200, 200)
    # patch.SetTexture(chrono.GetChronoDataPath() + "textures/tile4.jpg", 200, 200)
    # terrain.Initialize()
    # In our case, the ground_body is already part of the system.

    # 3. Set necessary default parameters such as positions, forces, and interactions.
    # (Already done for vehicle initial position, terrain friction/restitution)

    # Create the Irrlicht visualization application
    # This also handles the "driver system" for interactive control
    app = veh.ChVehicleIrrApp(uaz.GetVehicle(), "UAZBUS on Rigid Terrain")
    app.SetSkyBox()
    app.AddTypicalLights()
    
    # Set camera parameters
    # Arguments: chasePost, chaseDistance, chaseHeight
    app.SetChaseCamera(chrono.ChVectorD(0.0, 0.0, 0.75), 6.0, 0.5)
    app.SetChaseCameraState(veh.ChChaseCamera.Track) # Track the vehicle
    app.SetChaseCameraPosition(initial_pos - chrono.ChVectorD(-8,0,-3)) # Initial camera position

    # Initialize the driver system (part of ChVehicleIrrApp)
    # The ChIrrGuiDriver is created internally by ChVehicleIrrApp.
    # We can get a handle to it if needed, but app handles its lifecycle.
    # driver = app.GetDriver() # This would get the ChIrrGuiDriver
    # driver.SetSteeringDelta(0.05)
    # driver.SetThrottleDelta(0.05)
    # driver.SetBrakingDelta(0.1)

    # Set the time step for the simulation
    timestep = 0.01
    app.SetTimestep(timestep)


    # 4. Implement a simulation loop
    # Bind visualization assets
    app.AssetBindAll()
    app.AssetUpdateAll()

    print("Simulation started. Use keyboard controls for the UAZBUS:")
    print(" W/S: Throttle/Brake")
    print(" A/D: Steering Left/Right")
    print(" Q/E: Gear Up/Down (if applicable)")
    print(" Camera controls: Mouse or specific keys (see Irrlicht docs/ChVehicleIrrApp)")

    while app.GetDevice().run():
        # Get current simulation time
        time = system.GetChTime()

        # Start rendering the scene
        app.BeginScene(True, True, chrono.ChColor(0.2, 0.2, 0.7))
        app.DrawAll()
        
        # Get driver inputs (from keyboard/mouse via Irrlicht)
        # This is handled implicitly by ChVehicleIrrApp's driver
        driver_inputs = app.GetDriver().GetInputs()

        # Synchronize systems
        # The ChVehicleIrrApp's DoStep() handles synchronization and advancement
        # of vehicle, terrain (if it's a veh.Terrain object), and driver.
        # However, if we have custom terrain like our ground_body,
        # vehicle.Synchronize still needs it.
        
        # For explicit synchronization (often good for clarity):
        # driver_app = app.GetDriver() # Get the ChIrrGuiDriver
        # driver_app.Synchronize(time)
        # uaz.Synchronize(time, driver_inputs, terrain) # Pass the RigidTerrain object
                                                      # For custom body terrain, vehicle interacts via system contacts
        # terrain.Synchronize(time) # If using veh.RigidTerrain

        # Advance the simulation
        # app.DoStep() handles:
        # 1. driver.Advance(timestep)
        # 2. uaz.Advance(timestep)
        # 3. terrain.Advance(timestep) (if it's a veh.Terrain object)
        # 4. system.DoStepDynamics(timestep)
        # 5. Real-time synchronization (waits if simulation is too fast)
        app.DoStep() # This advances the ChSystem and vehicle related components

        # End rendering the scene
        app.EndScene()

    # Simulation cleanup is handled by Python's garbage collector for PyChrono objects
    print("Simulation ended.")

if __name__ == "__main__":
    main()