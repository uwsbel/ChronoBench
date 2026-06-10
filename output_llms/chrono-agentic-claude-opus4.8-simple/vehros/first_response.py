import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.ros as chros

def main():
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())                 # core data path
    veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')             # vehicle data path

    hmmwv = veh.HMMWV_Full()                                             # full HMMWV model
    hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)                   # rigid-terrain default
    hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)               # no chassis collision shape
    hmmwv.SetChassisFixed(False)                                        # chassis must be free to move
    hmmwv.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 1.6), chrono.ChQuaterniond(1, 0, 0, 0)))
    hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)                      # engine type
    hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)  # transmission type
    hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)                          # all-wheel drive
    hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)                 # steering type
    hmmwv.SetTireType(veh.TireModelType_TMEASY)                          # TMeasy tire model
    hmmwv.SetTireStepSize(1e-3)                                          # tire integration step
    hmmwv.Initialize()                                                   # build the vehicle

    system = hmmwv.GetSystem()                                          # the wrapper-owned system
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # required with contact
    print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())              # diagnostic banner

    hmmwv.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetTireVisualizationType(veh.VisualizationType_PRIMITIVES)

    terrain = veh.RigidTerrain(system)                                  # rigid terrain on the shared system
    patch_mat = chrono.ChContactMaterialNSC()                          # NSC contact material
    patch_mat.SetFriction(0.9)                                         # terrain friction
    patch_mat.SetRestitution(0.01)                                    # terrain restitution
    patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 100, 100)    # 100x100 flat patch
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()                                              # finalize terrain

    driver = veh.ChDriver(hmmwv.GetVehicle())                         # base driver, fed over ROS
    driver.Initialize()                                               # initialize driver

    ros_manager = chros.ChROSPythonManager()                          # python ROS manager
    ros_manager.RegisterHandler(chros.ChROSClockHandler())            # /clock first
    ros_manager.RegisterHandler(chros.ChROSDriverInputsHandler(25, driver, "~/input/driver_inputs"))  # SUBSCRIBE throttle/steer/brake
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(25, hmmwv.GetChassisBody(), "~/output/hmmwv/state"))  # publish chassis state
    ros_manager.Initialize()                                          # initialize once, after registration

    hmmwv.GetVehicle().EnableRealtime(True)                           # wall-clock pacing
    step_size = 1e-3                                                  # integration step
    time_end = 30                                                     # simulation end time

    time = 0
    while time < time_end:                                            # headless ROS loop
        time = system.GetChTime()                                    # current sim time

        driver_inputs = driver.GetInputs()                          # inputs (set by ROS subscriber)

        driver.Synchronize(time)                                    # synchronize driver
        terrain.Synchronize(time)                                   # synchronize terrain
        hmmwv.Synchronize(time, driver_inputs, terrain)             # synchronize vehicle

        driver.Advance(step_size)                                   # advance driver
        terrain.Advance(step_size)                                  # advance terrain
        hmmwv.Advance(step_size)                                    # advance vehicle (steps the system)

        if not ros_manager.Update(time, step_size):                 # publish to ROS, break on shutdown
            break

if __name__ == "__main__":
    main()
