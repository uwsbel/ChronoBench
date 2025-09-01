import pychrono as ch
import pychrono.vehicle as veh
import pychrono.ros as chros
import pychrono.irrlicht as chronoirr  # Instruction 1: Added Irrlicht import

def main():
    # Instruction 2: Set vehicle data path
    veh.SetDataPath(ch.GetChronoDataPath() + 'vehicle/')

    # Create and configure HMMWV vehicle
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(ch.ChContactMethod_NSC)
    hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
    hmmwv.SetChassisFixed(False)
    hmmwv.SetInitPosition(ch.ChCoordsysd(ch.ChVector3d(0, 0, 1.6), ch.ChQuaterniond(1, 0, 0, 0)))
    hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
    hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
    hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
    hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
    hmmwv.SetTireType(veh.TireModelType_TMEASY)
    hmmwv.SetTireStepSize(1e-3)

    # Instruction 3: Enhanced visualization settings
    hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetSteeringVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

    hmmwv.Initialize()

    # Create terrain with texture
    terrain = veh.RigidTerrain(hmmwv.GetSystem())
    patch_mat = ch.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch = terrain.AddPatch(patch_mat, ch.CSYSNORM, 100.0, 100.0)
    
    # Instruction 4: Add terrain texture
    texture = ch.ChTexture()
    texture.SetTextureFilename(veh.GetDataFile("terrain/textures/tile4.jpg"))
    texture.SetTextureScale(100, 100)
    patch.GetGroundBody().GetVisualShape(0).SetTexture(texture)
    
    terrain.Initialize()

    # Create driver system
    driver = veh.ChDriver(hmmwv.GetVehicle())
    # Removed incorrect driver.Initialize()

    # ROS configuration
    ros_manager = chros.ChROSPythonManager()
    ros_manager.RegisterHandler(chros.ChROSClockHandler())
    ros_manager.RegisterHandler(
        chros.ChROSDriverInputsHandler(25, driver, "~/input/driver_inputs"))
    ros_manager.RegisterHandler(
        chros.ChROSBodyHandler(25, hmmwv.GetChassisBody(), "~/output/hmmwv/state"))
    ros_manager.Initialize()

    # Instruction 5: Setup Irrlicht visualization
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(hmmwv.GetSystem())
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle('HMMWV Simulation')
    vis.Initialize()
    vis.AddCamera(ch.ChVector3d(0, 6, 2), ch.ChVector3d(0, 0, 0))
    vis.AddTypicalLights()

    # Simulation loop
    time = 0
    time_step = 1e-3
    time_end = 30

    # Corrected real-time enable call
    hmmwv.EnableRealtime(True)

    while time < time_end:
        # Check if visualization window closed
        if not vis.Run():
            break

        time = hmmwv.GetSystem().GetChTime()

        # Get and process driver inputs
        driver_inputs = driver.GetInputs()
        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)

        # Advance simulation
        driver.Advance(time_step)
        terrain.Advance(time_step)
        hmmwv.Advance(time_step)

        # Update ROS
        if not ros_manager.Update(time, time_step):
            break

        # Instruction 6: Render visualization
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

if __name__ == "__main__":
    main()