import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.ros as chros


class HMMWVDriver(veh.ChDriver):                                       # scripted ROS-overridable driver
    def __init__(self, vehicle):
        super().__init__(vehicle)                                      # bind to the vehicle

    def Synchronize(self, time):
        self.SetThrottle(0.6)                                          # steady forward throttle
        self.SetSteering(0.3 * math.sin(0.5 * time))                   # gentle sinusoidal steer
        self.SetBraking(0.0)                                           # no braking


def main():
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())              # locate bundled Chrono assets
    veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')          # locate vehicle data files

    step_size = 1e-3                                                   # integration step (s)
    sim_end = 10.0                                                     # total simulated time (s)

    init_loc = chrono.ChVector3d(0, 0, 0.5)                           # chassis spawn (HMMWV ref height)
    init_rot = chrono.QuatFromAngleZ(0)                               # facing +X

    hmmwv = veh.HMMWV_Full()                                          # full HMMWV catalog model
    hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)               # NSC for rigid terrain
    hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)            # no chassis collision mesh
    hmmwv.SetChassisFixed(False)                                     # MANDATORY — chassis must move
    hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))   # initial pose
    hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)                  # prompt: engine type
    hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)  # matching transmission
    hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)                     # all-wheel drive
    hmmwv.SetTireType(veh.TireModelType_TMEASY)                     # prompt: tire model
    hmmwv.SetTireStepSize(step_size)                               # tire integration step
    hmmwv.Initialize()                                             # build the vehicle subsystems

    hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)        # chassis mesh
    hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)  # suspension
    hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)    # steering
    hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)          # wheels
    hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)           # tires

    system = hmmwv.GetSystem()                                     # take the wrapper-owned system
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED for contact
    print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())         # truth's vehicle-mass banner

    terrain = veh.RigidTerrain(system)                            # flat rigid terrain
    patch_mat = chrono.ChContactMaterialNSC()                     # NSC contact material
    patch_mat.SetFriction(0.9)                                    # terrain friction
    patch_mat.SetRestitution(0.01)                               # terrain restitution
    patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 100.0, 100.0)  # 100x100 m ground patch
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)  # tile texture
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))               # terrain color
    terrain.Initialize()                                         # finalize terrain

    driver = HMMWVDriver(hmmwv.GetVehicle())                     # scripted driver (ROS may override)
    driver.Initialize()                                         # initialize the driver

    hmmwv.GetChassisBody().SetName("chassis")                   # name the chassis for ROS topics

    ros_manager = chros.ChROSPythonManager()                   # ROS2 bridge manager
    ros_manager.RegisterHandler(chros.ChROSClockHandler())     # /clock first — sim-time sync
    ros_manager.RegisterHandler(                               # subscribe driver throttle/steer/brake
        chros.ChROSDriverInputsHandler(25, driver, "~/input/driver_inputs"))
    ros_manager.RegisterHandler(                               # publish chassis pose/twist
        chros.ChROSBodyHandler(25, hmmwv.GetChassisBody(), "~/output/hmmwv/state"))
    ros_manager.Initialize()                                    # finalize ROS after all handlers

    render_every = max(1, round(1.0 / (50.0 * step_size)))               # render cadence (steps/frame)
    step_number = 0                                                       # physics-step counter

    while True:                                                  # headless real-time ROS loop
        time = system.GetChTime()                              # current sim time
        if time >= sim_end:                                    # stop at end time
            break


        driver_inputs = driver.GetInputs()                    # current driver inputs

        driver.Synchronize(time)                              # update driver state
        terrain.Synchronize(time)                             # update terrain
        hmmwv.Synchronize(time, driver_inputs, terrain)       # feed inputs + terrain to vehicle

        driver.Advance(step_size)                             # advance driver
        terrain.Advance(step_size)                            # advance terrain
        hmmwv.Advance(step_size)                              # advances the wrapper-owned system

        if not ros_manager.Update(time, step_size):           # publish/subscribe ROS — break on shutdown
            break

        step_number += 1                                      # next step


if __name__ == "__main__":
    main()
