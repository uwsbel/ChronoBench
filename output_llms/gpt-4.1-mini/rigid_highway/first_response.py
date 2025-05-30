import pychrono as chrono
import pychrono_irrlicht as chronoirr
import pychrono_vehicle as veh


def main():
    # -----------------------------------------------------
    # 1. Initialize PyChrono environment and core components
    # -----------------------------------------------------

    chrono.SetChronoDataPath(chrono.GetChronoDataPath())  # Ensure data path is set

    # Set simulation parameters
    step_size = 1.0 / 50  # seconds (50 FPS)
    tire_type = veh.TireModelType_TMEASY

    # -----------------------------------------------------
    # 2. Add required physical systems and objects
    # -----------------------------------------------------

    # Create the HMMWV vehicle with mesh visualization, TMEASY tire, and contact method
    contact_method = chrono.ChContactMethod_NSC  # Non-smooth contact
    initial_pos = chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.5), chrono.Q_from_AngZ(0))
    vehicle = veh.HMMWV_Full(
        contact_method=contact_method,
        fixed=False,
        drive_type=veh.DrivelineTypeWV_FWD,
        chassis_collision_type=veh.ChassisCollisionType_NONE,  # We'll rely on mesh collision in terrain and tires
    )

    vehicle.SetInitPosition(initial_pos)
    vehicle.SetChassisVisualizationType(veh.VisualizationTypeMESH)
    vehicle.SetSuspensionVisualizationType(veh.VisualizationTypeMESH)
    vehicle.SetSteeringVisualizationType(veh.VisualizationTypeMESH)
    vehicle.SetWheelVisualizationType(veh.VisualizationTypeMESH)
    vehicle.SetTireType(tire_type)
    vehicle.SetTireVisualizationType(veh.VisualizationTypeMESH)

    vehicle.Initialize()

    # Create the custom mesh terrain
    terrain = veh.Terrain(contact_method)

    # Set collision mesh
    terrain.InitializeMesh(terrain_file="Highway_col.obj")
    # Set visualization mesh
    terrain.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"))  # Use some texture
    terrain.SetTextureScale(8, 8)
    terrain.SetMesh(visual_mesh_file="Highway_vis.obj", collide_mesh_file="Highway_col.obj")
    terrain.Initialize()

    # For the function above, there isn't a direct SetMesh method that takes both visual and collision mesh.
    # Instead, we need to do the collision and visualization manually because Terrain base class does not provide direct helpers:
    # Instead, we do this manually:

    # Remove the above call and do a custom terrain creation instead:
    # But the provided instruction is to use a custom mesh with collision and visual meshes.

    # So, create a ChBody for terrain to hold the collision and visual shapes
    terrain_body = chrono.ChBody()
    terrain_body.SetBodyFixed(True)

    # Load collision mesh
    collision_mesh = chrono.ChTriangleMeshShape()
    collision_mesh.SetFilename("Highway_col.obj")
    collision_mesh.SetStatic(True)
    terrain_body.AddAsset(collision_mesh)

    # Create collision shape from the mesh
    trimesh = chrono.ChTriangleMeshConnected()
    trimesh.LoadWavefrontMesh("Highway_col.obj", False, False)
    collision_shape = chrono.ChTriangleMeshShape()
    collision_shape.SetMesh(trimesh)
    terrain_body.AddAsset(collision_shape)
    terrain_body.GetCollisionModel().ClearModel()
    terrain_body.GetCollisionModel().AddTriangleMesh(
        trimesh, False, False, chrono.ChVectorD(0, 0, 0), chrono.ChMatrix33D(chrono.ChVectorD(1, 1, 1)), 0.01)
    terrain_body.GetCollisionModel().BuildModel()
    terrain_body.SetCollide(True)

    # Load visualization mesh
    vis_mesh = chrono.ChTriangleMeshShape()
    vis_mesh.SetFilename("Highway_vis.obj")
    vis_mesh.SetStatic(True)
    terrain_body.AddAsset(vis_mesh)

    # Add material to terrain for visualization
    terrain_material = chrono.ChVisualMaterial()
    terrain_material.SetKdTexture("../../data/terrain/textures/tile4.jpg")
    terrain_body.GetAssets().at(terrain_body.GetAssets().size()-1).SetMaterial(terrain_material)

    # Create the system where the vehicle and terrain reside
    system = vehicle.GetSystem()
    system.Add(terrain_body)

    # -----------------------------------------------------
    # 3. Set parameters: interactive driver system
    # -----------------------------------------------------

    # Create the Irrlicht visualization system
    vis = chronoirr.ChIrrApp(vehicle.GetSystem(), "HMMWV on Custom Mesh Terrain",
                             chronoirr.dimension2du(1280, 720))

    vis.AddTypicalSky()
    vis.AddTypicalLights()
    vis.AddTypicalCamera(chronoirr.vector3df(5, -10, 3), chronoirr.vector3df(0, 0, 0.5))
    vis.AssetBindAll()
    vis.AssetUpdateAll()

    # Create the driver system with interactive keyboard control
    driver = veh.ChInteractiveDriverIRR(vis)
    driver.SetInputDelay(0.1)   # delay before inputs apply
    driver.SetSteeringDelta(0.1)
    driver.SetThrottleDelta(0.1)
    driver.SetBrakingDelta(0.1)

    # Initialize the driver system
    driver.Initialize()

    # Simulation loop
    vis.SetStepManage(True)
    vis.SetTimestep(step_size)

    realtime_timer = chrono.ChRealtimeStepTimer()

    print("Start simulation")

    while vis.Run():
        vis.BeginScene()
        vis.DrawAll()

        # Get driver inputs
        steering = driver.GetSteering()
        throttle = driver.GetThrottle()
        braking = driver.GetBraking()

        # Apply inputs to vehicle systems
        vehicle.GetSteering().SetInput(steering)
        vehicle.GetPowertrain().SetThrottle(throttle)
        vehicle.GetBrakingControl().SetBrake(braking)

        # Synchronize modules
        time = vehicle.GetSystem().GetChTime()
        driver.Synchronize(time)
        terrain_body.SyncCollisionModels()

        vehicle.Synchronize(time, driver.GetInputs(), terrain_body)
        terrain_body.Update()

        # Advance simulation by step_size
        vehicle.Advance(step_size)
        driver.Advance(step_size)
        terrain_body.Advance(step_size)

        vis.EndScene()

        # Enforce real-time
        realtime_timer.Spin(step_size)


if __name__ == "__main__":
    main()