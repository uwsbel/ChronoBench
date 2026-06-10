import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr


# Vehicle data path
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# Simulation step sizes
step_size = 2e-3
tire_step_size = 1e-3


def add_visual_mesh_to_terrain(terrain, mesh_file):
    """Load a Wavefront OBJ mesh and attach it as a visual shape to the terrain ground body."""
    vis_mesh = chrono.ChTriangleMeshConnected()
    vis_mesh.LoadWavefrontMesh(mesh_file, False, True)

    vis_shape = chrono.ChVisualShapeTriangleMesh()
    vis_shape.SetMesh(vis_mesh)
    vis_shape.SetName("Highway visual mesh")
    vis_shape.SetMutable(False)

    terrain.GetGroundBody().AddVisualShape(vis_shape)


def main():
    # Create the HMMWV vehicle, set parameters, and initialize
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
    hmmwv.SetChassisFixed(False)

    # Updated initial position
    hmmwv.SetInitPosition(
        chrono.ChCoordsysd(
            chrono.ChVector3d(6, -70, 0.5),
            chrono.ChQuaterniond(1, 0, 0, 0)
        )
    )

    hmmwv.SetEngineType(veh.EngineModelType_SIMPLE)
    hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
    hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
    hmmwv.SetTireType(veh.TireModelType_TMEASY)
    hmmwv.SetTireStepSize(tire_step_size)

    hmmwv.Initialize()

    hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetSteeringVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

    # Required for mesh collision terrain
    hmmwv.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    # -------------------------------------------------------------------------
    # Terrain definition: single highway mesh collision patch
    # -------------------------------------------------------------------------
    terrain = veh.RigidTerrain(hmmwv.GetSystem())

    terrain_mat = chrono.ChContactMaterialNSC()
    terrain_mat.SetFriction(0.9)
    terrain_mat.SetRestitution(0.01)

    # Collision mesh:
    # veh.GetDataFile("terrain/...") resolves to:
    # <Chrono data path>/vehicle/terrain/...
    highway_collision_mesh = veh.GetDataFile("terrain/meshes/Highway_col.obj")

    # The final argument, 0.01, is the swept-sphere/contact thickness used for
    # triangle mesh collision robustness.
    terrain.AddPatch(
        terrain_mat,
        chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
        highway_collision_mesh,
        True,
        0.01
    )

    # Add separate visual mesh to the terrain ground body
    highway_visual_mesh = veh.GetDataFile("terrain/meshes/Highway_vis.obj")
    add_visual_mesh_to_terrain(terrain, highway_visual_mesh)

    terrain.Initialize()

    # -------------------------------------------------------------------------
    # Irrlicht visualization
    # -------------------------------------------------------------------------
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle("HMMWV Highway Terrain Demo")
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 0.75), 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    vis.AttachVehicle(hmmwv.GetVehicle())

    # Interactive driver
    driver = veh.ChInteractiveDriverIRR(vis)
    driver.SetSteeringDelta(0.02)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.06)
    driver.Initialize()

    hmmwv.GetVehicle().EnableRealtime(True)

    while vis.Run():
        time = hmmwv.GetSystem().GetChTime()

        # Draw scene
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        # Get driver inputs
        driver_inputs = driver.GetInputs()

        # Synchronize modules
        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        # Advance simulation
        driver.Advance(step_size)
        terrain.Advance(step_size)
        hmmwv.Advance(step_size)
        vis.Advance(step_size)

    return 0


if __name__ == "__main__":
    main()