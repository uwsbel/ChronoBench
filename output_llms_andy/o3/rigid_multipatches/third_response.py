import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

# -------------------------------------------------------------------------
# Global settings
# -------------------------------------------------------------------------
# Chrono data path for the vehicle module
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Simulation step sizes
step_size      = 2e-3     # main integrator step
tire_step_size = 1e-3     # finer tyre step (passed to the vehicle)


def main():
    # ---------------------------------------------------------------------
    # Create the HMMWV vehicle and initialise it
    # ---------------------------------------------------------------------
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
    hmmwv.SetChassisFixed(False)
    hmmwv.SetInitPosition(
        chrono.ChCoordsysd(chrono.ChVector3d(-10, -2, 0.6), chrono.ChQuaterniond(1, 0, 0, 0))
    )
    hmmwv.SetEngineType(veh.EngineModelType_SIMPLE)
    hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
    hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
    hmmwv.SetTireType(veh.TireModelType_TMEASY)
    hmmwv.SetTireStepSize(tire_step_size)
    hmmwv.Initialize()

    # Visualisation types
    hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetSteeringVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

    # Use Bullet for collision detection
    hmmwv.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    # ---------------------------------------------------------------------
    # Create a rigid terrain composed of several patches
    # ---------------------------------------------------------------------
    terrain = veh.RigidTerrain(hmmwv.GetSystem())

    # ---- Patch 1 : flat square -------------------------------------------------
    patch1_mat = chrono.ChContactMaterialNSC()
    patch1_mat.SetFriction(0.9)
    patch1_mat.SetRestitution(0.01)
    # POSITION CHANGED  (-20, 5, 0)
    patch1 = terrain.AddPatch(
        patch1_mat,
        chrono.ChCoordsysd(chrono.ChVector3d(-20,  5, 0), chrono.QUNIT),
        32, 20
    )
    patch1.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    patch1.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 20, 20)

    # ---- Patch 2 : flat rectangle ---------------------------------------------
    patch2_mat = chrono.ChContactMaterialNSC()
    patch2_mat.SetFriction(0.9)
    patch2_mat.SetRestitution(0.01)
    # POSITION CHANGED  (20, -5, 0.2)
    patch2 = terrain.AddPatch(
        patch2_mat,
        chrono.ChCoordsysd(chrono.ChVector3d( 20, -5, 0.2), chrono.QUNIT),
        32, 30
    )
    patch2.SetColor(chrono.ChColor(1.0, 0.5, 0.5))
    patch2.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 20, 20)

    # ---- Patch 3 : OBJ mesh ----------------------------------------------------
    patch3_mat = chrono.ChContactMaterialNSC()
    patch3_mat.SetFriction(0.9)
    patch3_mat.SetRestitution(0.01)
    # POSITION CHANGED  (5, -45, 0)
    patch3 = terrain.AddPatch(
        patch3_mat,
        chrono.ChCoordsysd(chrono.ChVector3d( 5, -45, 0), chrono.QUNIT),
        veh.GetDataFile("terrain/meshes/bump.obj")
    )
    patch3.SetColor(chrono.ChColor(0.5, 0.5, 0.8))
    patch3.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 6.0, 6.0)

    # ---- Patch 4 : height-map --------------------------------------------------
    patch4_mat = chrono.ChContactMaterialNSC()
    patch4_mat.SetFriction(0.9)
    patch4_mat.SetRestitution(0.01)
    # POSITION CHANGED  (10, 40, 0)
    patch4 = terrain.AddPatch(
        patch4_mat,
        chrono.ChCoordsysd(chrono.ChVector3d(10, 40, 0), chrono.QUNIT),
        veh.GetDataFile("terrain/height_maps/bump64.bmp"),
        64.0, 64.0,      # size X, size Y (m)
        0.0, 3.0         # height offset, height scale (m)
    )
    patch4.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 6.0, 6.0)

    # Finalise terrain
    terrain.Initialize()

    # ---------------------------------------------------------------------
    # Irrlicht visualisation system
    # ---------------------------------------------------------------------
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('HMMWV Rigid Terrain Demo')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 0.75), 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    vis.AttachVehicle(hmmwv.GetVehicle())

    # ---------------------------------------------------------------------
    # Driver (keyboard)
    # ---------------------------------------------------------------------
    driver = veh.ChInteractiveDriverIRR(vis)
    driver.SetSteeringDelta(0.02)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.06)
    driver.Initialize()

    # Enable real-time synchronization in the vehicle module
    hmmwv.GetVehicle().EnableRealtime(True)

    # ---------------------------------------------------------------------
    # Simulation loop
    # ---------------------------------------------------------------------
    while vis.Run():
        sim_time = hmmwv.GetSystem().GetChTime()

        # Render
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        # Driver inputs
        driver_inputs = driver.GetInputs()

        # Module synchronisation
        driver.Synchronize(sim_time)
        terrain.Synchronize(sim_time)
        hmmwv.Synchronize(sim_time, driver_inputs, terrain)
        vis.Synchronize(sim_time, driver_inputs)

        # Advance all modules
        driver.Advance(step_size)
        terrain.Advance(step_size)
        hmmwv.Advance(step_size)
        vis.Advance(step_size)

    return 0


# Execute only when run as a script
if __name__ == '__main__':
    main()