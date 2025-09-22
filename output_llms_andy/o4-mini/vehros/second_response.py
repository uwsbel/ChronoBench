import pychrono as ch
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import pychrono.ros as chros

def main():
    # ------------------------------------------------------------------------
    # 1) SETUP THE VEHICLE DATA PATH
    # ------------------------------------------------------------------------
    # This tells the vehicle subsystem where to find its JSONs, meshes, etc.
    veh.SetDataPath(ch.GetChronoDataPath() + "vehicle/")

    # ------------------------------------------------------------------------
    # 2) CREATE AND INITIALIZE THE HMMWV VEHICLE
    # ------------------------------------------------------------------------
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(ch.ChContactMethod_NSC)
    hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
    hmmwv.SetChassisFixed(False)
    hmmwv.SetInitPosition(ch.ChCoordsysd(
        ch.ChVector3d(0, 0, 1.6),
        ch.ChQuaterniond(1, 0, 0, 0)))
    hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
    hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
    hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
    hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
    hmmwv.SetTireType(veh.TireModelType_TMEASY)
    hmmwv.SetTireStepSize(1e-3)

    # --- VISUALIZATION SETTINGS ON THE VEHICLE ---
    #    (show meshes for chassis & wheels, primitives for suspension & steering, mesh tires)
    hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

    hmmwv.Initialize()

    # ------------------------------------------------------------------------
    # 3) CREATE AND INITIALIZE THE TERRAIN
    # ------------------------------------------------------------------------
    terrain = veh.RigidTerrain(hmmwv.GetSystem())

    # NOTE: for NSC contact surfaces the correct class is ChMaterialSurfaceNSC
    patch_mat = ch.ChMaterialSurfaceNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)

    # Add a single rectangular patch, centered at the global origin, size 100×100
    patch = terrain.AddPatch(patch_mat,
                             ch.ChCoordsysd(),  # identity frame = (0,0,0), no rotation
                             100.0, 100.0)

    # Apply a tiling texture to the terrain
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 100, 100)

    terrain.Initialize()

    # ------------------------------------------------------------------------
    # 4) CREATE THE DRIVER (ROS‐BASED) AND THE ROS MANAGER
    # ------------------------------------------------------------------------
    driver = veh.ChDriver(hmmwv.GetVehicle())
    driver.Initialize()

    ros_manager = chros.ChROSPythonManager()
    ros_manager.RegisterHandler(chros.ChROSClockHandler())
    ros_manager.RegisterHandler(
        chros.ChROSDriverInputsHandler(25, driver, "~/input/driver_inputs"))
    ros_manager.RegisterHandler(
        chros.ChROSBodyHandler(25, hmmwv.GetChassisBody(), "~/output/hmmwv/state"))
    ros_manager.Initialize()

    # ------------------------------------------------------------------------
    # 5) SETUP IRRLICHT FOR VISUALIZATION
    # ------------------------------------------------------------------------
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("HMMWV + ROS + Irrlicht")
    vis.Initialize()
    vis.AddLogo()
    vis.AddSkyBox()
    vis.AddTypicalLights()
    # Tell Irrlicht to draw all bodies in the Chrono system
    vis.AttachSystem(hmmwv.GetSystem())

    # ------------------------------------------------------------------------
    # 6) RUN THE SIMULATION
    # ------------------------------------------------------------------------
    time       = 0.0
    time_step  = 1e-3
    time_end   = 30.0

    # Run in (approximately) real time
    hmmwv.EnableRealtime(True)

    # Main loop: advance physics, publish/subscribe ROS, draw in Irrlicht
    while vis.Run() and time < time_end:
        # fetch the current simulation time
        time = hmmwv.GetSystem().GetChTime()

        # collect driver inputs (from ROS)
        inputs = driver.GetInputs()

        # synchronize all modules
        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, inputs, terrain)

        # render the scene
        vis.BeginScene()
        vis.DrawAll()
        vis.EndScene()

        # advance all modules by one timestep
        driver.Advance(time_step)
        terrain.Advance(time_step)
        hmmwv.Advance(time_step)

        # update ROS (publish clocks, vehicle state, etc.)
        if not ros_manager.Update(time, time_step):
            break

if __name__ == "__main__":
    main()