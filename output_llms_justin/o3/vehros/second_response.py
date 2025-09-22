# =============================================================================
#  HMMWV + ROS demo with Irrlicht visualisation
# =============================================================================

import pychrono as ch
from   pychrono import irrlicht as chronoirr          # 1. Irrlicht import
import pychrono.vehicle as veh
import pychrono.ros     as chros


def main():
    # -------------------------------------------------------------
    #  Chrono initialisation and general set–up
    # -------------------------------------------------------------
    ch.SetChronoDataPath(ch.GetChronoDataPath())                 # make sure core data path is known
    veh.SetDataPath(ch.GetChronoDataPath() + 'vehicle/')         # 2. vehicle data path

    contact_method = ch.ChContactMethod_NSC

    # -------------------------------------------------------------
    #  Create the HMMWV full vehicle
    # -------------------------------------------------------------
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(contact_method)

    # correct enumeration names
    hmmwv.SetChassisCollisionType(veh.ChassisCollisionType_NONE)
    hmmwv.SetChassisFixed(False)

    init_loc  = ch.ChVectorD(0.0, 0.0, 1.6)
    init_rot  = ch.ChQuaternionD(1.0, 0.0, 0.0, 0.0)
    hmmwv.SetInitPosition(ch.ChCoordsysD(init_loc, init_rot))

    hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
    hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
    hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
    hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
    hmmwv.SetTireType(veh.TireModelType_TMEASY)
    hmmwv.SetTireStepSize(1e-3)

    # 3. richer visualisation for individual subsystems
    hmmwv.SetChassisVisualizationType     (veh.VisualizationType_PRIMITIVES)
    hmmwv.SetSuspensionVisualizationType  (veh.VisualizationType_PRIMITIVES)
    hmmwv.SetSteeringVisualizationType    (veh.VisualizationType_PRIMITIVES)
    hmmwv.SetWheelVisualizationType       (veh.VisualizationType_PRIMITIVES)
    hmmwv.SetTireVisualizationType        (veh.VisualizationType_MESH)

    hmmwv.Initialize()

    # -------------------------------------------------------------
    #  Terrain
    # -------------------------------------------------------------
    terrain = veh.RigidTerrain(hmmwv.GetSystem())
    patch_mat = ch.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)

    patch = terrain.AddPatch(
        patch_mat,
        ch.ChCoordsysD(ch.ChVectorD(0.0, 0.0, 0.0), ch.ChQuaternionD(1.0, 0.0, 0.0, 0.0)),
        100.0, 100.0)

    # 4. give the patch a texture
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 100, 100)
    terrain.Initialize()

    # -------------------------------------------------------------
    #  Driver (placeholder, no user interaction)
    # -------------------------------------------------------------
    driver = veh.ChDriver(hmmwv.GetVehicle())
    driver.Initialize()

    # -------------------------------------------------------------
    #  ROS manager
    # -------------------------------------------------------------
    ros_manager = chros.ChROSPythonManager()
    ros_manager.RegisterHandler(chros.ChROSClockHandler())
    ros_manager.RegisterHandler(
        chros.ChROSDriverInputsHandler(25, driver, "~/input/driver_inputs"))
    ros_manager.RegisterHandler(
        chros.ChROSBodyHandler  (25, hmmwv.GetChassisBody(), "~/output/hmmwv/state"))
    ros_manager.Initialize()

    # -------------------------------------------------------------
    #  5. Irrlicht run-time visualisation
    # -------------------------------------------------------------
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(hmmwv.GetSystem())
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("HMMWV + ROS demo")
    vis.Initialize()
    vis.AddLogo()
    vis.AddSkyBox()
    vis.AddTypicalLights()
    vis.AddTypicalCamera(ch.ChVectorD(0.0, -6.0, 3.0))
    vis.SetCameraVertical(ch.ChVectorD(0, 0, 1))

    # -------------------------------------------------------------
    #  Simulation loop
    # -------------------------------------------------------------
    time_step   = 1e-3
    time_end    = 30.0
    render_step = 10          # render every Nth integration step
    out_step    = 0

    hmmwv.GetVehicle().EnableRealtime(True)

    while vis.Run() and (hmmwv.GetSystem().GetChTime() < time_end):
        time = hmmwv.GetSystem().GetChTime()

        # ----- module synchronisation
        driver_inputs = driver.GetInputs()
        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)

        # ----- advance state
        driver.Advance(time_step)
        terrain.Advance(time_step)
        hmmwv.Advance(time_step)

        # ----- ROS publishing
        if not ros_manager.Update(time, time_step):
            break

        # 6. Irrlicht rendering
        if out_step % render_step == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
        out_step += 1

    # ------------------------------------------------------------------
    ros_manager.Shutdown()


# ------------------------------------------------------------------------------
if __name__ == "__main__":
    main()