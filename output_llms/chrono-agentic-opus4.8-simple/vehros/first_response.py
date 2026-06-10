import pychrono as ch
import pychrono.vehicle as veh
import pychrono.ros as chros

def main():
    # Anchor the bundled Chrono + vehicle data paths so catalog assets resolve.
    ch.SetChronoDataPath(ch.GetChronoDataPath())                    # locate bundled Chrono assets
    veh.SetDataPath(ch.GetChronoDataPath() + 'vehicle/')           # locate vehicle data files

    # Create the HMMWV vehicle and set its parameters.
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(ch.ChContactMethod_NSC)                  # NSC contact for rigid terrain
    hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)          # disable chassis collision
    hmmwv.SetChassisFixed(False)                                    # chassis must be movable
    hmmwv.SetInitPosition(ch.ChCoordsysd(ch.ChVector3d(0, 0, 1.6), ch.ChQuaterniond(1, 0, 0, 0)))  # spawn pose
    hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)                # shaft-based engine model
    hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)  # automatic shaft transmission
    hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)                    # all-wheel drive
    hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)          # pitman-arm steering
    hmmwv.SetTireType(veh.TireModelType_TMEASY)                    # TMeasy tire model
    hmmwv.SetTireStepSize(1e-3)                                    # tire sub-step size
    hmmwv.Initialize()                                             # build the vehicle subsystems
    system = hmmwv.GetSystem()                                     # wrapper-owned ChSystem
    system.SetCollisionSystemType(ch.ChCollisionSystem.Type_BULLET)  # Bullet collision (contact scene)
    print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())          # report total vehicle mass

    # Create the rigid terrain the vehicle drives on.
    terrain = veh.RigidTerrain(hmmwv.GetSystem())
    patch_mat = ch.ChContactMaterialNSC()                          # terrain contact material
    patch_mat.SetFriction(0.9)                                     # terrain friction coefficient
    patch_mat.SetRestitution(0.01)                                 # terrain restitution
    terrain.AddPatch(patch_mat, ch.CSYSNORM, 100.0, 100.0)        # flat 100x100 m patch at origin
    terrain.Initialize()                                          # build the terrain

    # Create and initialize the driver system (inputs arrive over ROS).
    driver = veh.ChDriver(hmmwv.GetVehicle())
    driver.Initialize()                                           # initialize the driver

    # Create the ROS manager and register the communication handlers.
    ros_manager = chros.ChROSPythonManager()
    ros_manager.RegisterHandler(chros.ChROSClockHandler())        # /clock — sync ROS time to sim
    ros_manager.RegisterHandler(chros.ChROSDriverInputsHandler(25, driver, "~/input/driver_inputs"))  # subscribe driver inputs
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(25, hmmwv.GetChassisBody(), "~/output/hmmwv/state"))  # publish vehicle state
    ros_manager.Initialize()                                      # finalize ROS handler registration


    # Run the simulation loop.
    time = 0
    time_step = 1e-3                                               # integration step
    time_end = 30                                                 # total simulation duration (s)

    hmmwv.GetVehicle().EnableRealtime(True)                        # pace the sim to wall-clock
    while time < time_end:
        time = hmmwv.GetSystem().GetChTime()                      # current sim time

        # Read driver inputs and synchronize the subsystems.
        driver_inputs = driver.GetInputs()
        driver.Synchronize(time)                                  # synchronize the driver
        terrain.Synchronize(time)                                 # synchronize the terrain
        hmmwv.Synchronize(time, driver_inputs, terrain)           # synchronize the vehicle


        # Advance all subsystems by one timestep.
        driver.Advance(time_step)
        terrain.Advance(time_step)
        hmmwv.Advance(time_step)

        # Update the ROS manager to publish/subscribe this step.
        if not ros_manager.Update(time, time_step):
            break                                                 # stop if the ROS layer shut down


if __name__ == "__main__":
    main()
