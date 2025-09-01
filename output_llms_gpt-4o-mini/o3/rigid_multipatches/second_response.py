#
#  HMMWV on a highway mesh – simplified rigid-terrain example
#

import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr          # the Irrlicht run-time visualization

# --------------------------------------------------------------------------
# Global settings
# --------------------------------------------------------------------------
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')   # vehicle data
STEP_SIZE       = 2e-3                                     # integrator step
TIRE_STEP_SIZE  = 1e-3                                     # tire sub-stepping


# =============================================================================
def main():
    # ----------------------------------------------------------------------
    # Create the HMMWV vehicle
    # ----------------------------------------------------------------------
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
    hmmwv.SetChassisFixed(False)

    init_loc = chrono.ChVectorD( 6, -70, 0.5 )             # 1. new position
    init_rot = chrono.ChQuaternionD(1, 0, 0, 0)
    hmmwv.SetInitPosition(chrono.ChCoordsysD(init_loc, init_rot))

    hmmwv.SetEngineType(veh.EngineModelType_SIMPLE)
    hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)

    # Driveline enumeration changed in recent Chrono versions – try both names
    try:
        hmmwv.SetDriveType(veh.DrivelineTypeWV.AWD)
    except AttributeError:
        hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)

    hmmwv.SetTireType(veh.TireModelType_TMEASY)
    hmmwv.SetTireStepSize(TIRE_STEP_SIZE)
    hmmwv.Initialize()

    hmmwv.SetChassisVisualizationType     (veh.VisualizationType_MESH)
    hmmwv.SetSuspensionVisualizationType  (veh.VisualizationType_MESH)
    hmmwv.SetSteeringVisualizationType    (veh.VisualizationType_MESH)
    hmmwv.SetWheelVisualizationType       (veh.VisualizationType_MESH)
    hmmwv.SetTireVisualizationType        (veh.VisualizationType_MESH)

    sys = hmmwv.GetSystem()
    sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    # ----------------------------------------------------------------------
    # Rigid terrain – a SINGLE patch defined by a triangular collision mesh
    # ----------------------------------------------------------------------
    terrain = veh.RigidTerrain(sys)

    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)

    coll_mesh = veh.GetDataFile("terrain/meshes/Highway_col.obj")
    patch_cs  = chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT)

    # The last argument is the sweep thickness for collision detection (0.01 m)
    patch = terrain.AddPatch(patch_mat, patch_cs, coll_mesh, 0.01)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.8))
    patch.SetTexture(veh.GetDataFile("terrain/textures/road.jpg"), 1.0, 1.0)

    # ----- add a separate VISUAL mesh
    vis_mesh = chrono.ChTriangleMeshConnected()
    vis_mesh.LoadWavefrontMesh(veh.GetDataFile("terrain/meshes/Highway_vis.obj"),
                               False,  # no vertex welding
                               True)   # automatic normal generation

    vis_shape = chrono.ChVisualShapeTriangleMesh(vis_mesh)
    vis_shape.SetName("Highway_vis")
    vis_shape.SetBackfaceCull(True)
    vis_shape.SetColor(chrono.ChColor(0.4, 0.4, 0.4))

    # Attach the visual shape to the ground body that belongs to this patch
    ground_body = patch.GetGroundBody() if hasattr(patch, "GetGroundBody") \
                                        else patch.GetGround()
    ground_body.AddVisualShape(vis_shape)

    terrain.Initialize()

    # ----------------------------------------------------------------------
    # Irrlicht run-time visualization
    # ----------------------------------------------------------------------
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('HMMWV – Highway mesh demo')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVectorD(0.0, 0.0, 0.75), 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    vis.AttachVehicle(hmmwv.GetVehicle())

    # ----------------------------------------------------------------------
    # Interactive driver (keyboard)
    # ----------------------------------------------------------------------
    driver = veh.ChInteractiveDriverIRR(vis)
    driver.SetSteeringDelta (0.02)
    driver.SetThrottleDelta (0.02)
    driver.SetBrakingDelta  (0.06)
    driver.Initialize()

    hmmwv.GetVehicle().EnableRealtime(True)

    # ----------------------------------------------------------------------
    # Simulation loop
    # ----------------------------------------------------------------------
    while vis.Run():
        time = sys.GetChTime()

        # Render
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        # Driver inputs
        driver_inputs = driver.GetInputs()

        # Synchronize modules
        driver.Synchronize (time)
        terrain.Synchronize(time)
        hmmwv.Synchronize  (time, driver_inputs, terrain)
        vis.Synchronize    (time, driver_inputs)

        # Advance simulation
        driver.Advance (STEP_SIZE)
        terrain.Advance(STEP_SIZE)
        hmmwv.Advance  (STEP_SIZE)
        vis.Advance    (STEP_SIZE)

    return 0


# =============================================================================
if __name__ == '__main__':
    main()