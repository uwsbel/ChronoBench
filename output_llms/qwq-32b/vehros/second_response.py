import pychrono as ch
import pychrono.vehicle as veh
import pychrono.ros as chros
from pychrono import irrlicht as chronoirr

def main():
    # Set the data path for vehicle assets
    veh.SetDataPath(ch.GetChronoDataPath() + 'vehicle/')

    # Create the HMMWV vehicle and set its parameters.
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(ch.ChContactMethod.NSC)  # Corrected enum syntax
    hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
    hmmwv.SetChassisFixed(False)
    hmmwv.SetInitPosition(ch.ChCoordsysd(ch.ChVectorD(0, 0, 1.6), ch.ChQuaternionD(1, 0, 0, 0)))
    hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
    hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
    hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
    hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
    hmmwv.SetTireType(veh.TireModelType_TMEASY)
    hmmwv.SetTireStepSize(1e-3)

    # Enhanced visualization settings
    hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetTireVisualizationType(veh.VisualizationType_NONE)
    hmmwv.Initialize()

    # Create and initialize terrain
    terrain = veh.RigidTerrain(hmmwv.GetSystem())
    patch_mat = ch.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch = terrain.AddPatch(patch_mat, ch.ChCoordsysd(), 100.0, 100.0)  # Fixed CSYS parameter
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 100, 100)  # Added texture
    terrain.Initialize()

    # Setup visualization
    visualiz = chronoirr.ChVisualSystemIrrlicht()
    visualiz.SetWindowSize(1024, 768)
    visualiz.SetWindowTitle('HMMWV Simulation')
    visualiz.SetSymbolsScale(0.002)
    visualiz.SetCameraPosition(0, 0, 5)
    visualiz.SetRenderingCamera(chronoirr.ChIrrCameraSmoothPositioned(
        hmmwv.GetChassisBody(), ch.Vec3(0, 0, 2), ch.Vec3(0, 1, 0), 5))
    visualiz.Initialize()
    visualiz.AddLightDirectional()
    hmmwv.GetVehicle().InitializeRenderInterface(visualiz)  # Ensure vehicle visuals are added
    terrain.GetGroundBody().SetCollide(True)  # Ensure terrain is visible

    # Initialize driver and ROS manager
    driver = veh.ChDriver(hmmwv.GetVehicle())
    driver.Initialize()
    
    ros_manager = chros.ChROSPythonManager()
    ros_manager.RegisterHandler(chros.ChROSClockHandler())
    ros_manager.RegisterHandler(chros.ChROSDriverInputsHandler(25, driver, "~/input/driver_inputs"))
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(25, hmmwv.GetChassisBody(), "~/output/hmmwv/state"))
    ros_manager.Initialize()

    # Simulation parameters
    time_step = 1e-3
    time_end = 30
    hmmwv.GetVehicle().EnableRealtime(True)

    # Main simulation loop
    while visualiz.Run() and hmmwv.GetSystem().GetChTime() < time_end:
        time = hmmwv.GetSystem().GetChTime()

        # Synchronize and advance systems
        driver_inputs = driver.GetInputs()
        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        
        driver.Advance(time_step)
        terrain.Advance(time_step)
        hmmwv.Advance(time_step)

        # Update ROS and render
        if not ros_manager.Update(time, time_step):
            break

        # Render visualization
        visualiz.BeginScene()
        visualiz.Render()
        visualiz.EndScene()
        key = visualiz.WaitKey(int(time_step * 1000))  # Frame delay based on time_step
        if key == ord('q'):
            break

if __name__ == "__main__":
    main()