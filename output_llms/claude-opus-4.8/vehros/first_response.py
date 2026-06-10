import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.ros as chros
import math


def main():
    # ------------------------------------------------------------------
    # 1. Initialize the PyChrono environment and core components
    # ------------------------------------------------------------------
    # Set the data path for vehicle models/resources
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())
    veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

    # Contact method used for the entire system (NSC or SMC)
    contact_method = chrono.ChContactMethod_NSC

    # ------------------------------------------------------------------
    # 2. Create and configure the HMMWV vehicle
    # ------------------------------------------------------------------
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(contact_method)

    # Chassis collision type
    hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
    hmmwv.SetChassisFixed(False)

    # Initial position and orientation of the vehicle
    init_loc = chrono.ChVector3d(0, 0, 0.5)
    init_rot = chrono.ChQuaterniond(1, 0, 0, 0)
    hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))

    # Powertrain / engine configuration
    hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
    hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)

    # Drivetrain (AWD/FWD/RWD)
    hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)

    # Tire model
    hmmwv.SetTireType(veh.TireModelType_TMEASY)
    hmmwv.SetTireStepSize(1e-3)

    # Initialize the vehicle
    hmmwv.Initialize()

    # Visualization types for vehicle subsystems
    hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

    # Convenience reference to the underlying Chrono system
    system = hmmwv.GetSystem()

    # ------------------------------------------------------------------
    # 3. Create the terrain with defined friction and restitution
    # ------------------------------------------------------------------
    terrain = veh.RigidTerrain(system)

    # Contact material parameters for the terrain
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)        # Friction coefficient
    patch_mat.SetRestitution(0.01)    # Restitution (bounciness)

    # Create a flat rectangular patch
    patch = terrain.AddPatch(
        patch_mat,
        chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
        200.0, 200.0  # Length, Width
    )
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)

    terrain.Initialize()

    # ------------------------------------------------------------------
    # 4. Initialize a driver system for vehicle control
    # ------------------------------------------------------------------
    driver = veh.ChDriver(hmmwv.GetVehicle())
    driver.Initialize()

    # ------------------------------------------------------------------
    # 5. Integrate ROS for communication
    # ------------------------------------------------------------------
    ros_manager = chros.ChROSPythonManager()

    # Handler for clock synchronization (publishes simulation time)
    clock_handler = chros.ChROSClockHandler()
    ros_manager.RegisterHandler(clock_handler)

    # Handler for receiving driver inputs over ROS
    driver_inputs_rate = 25  # Hz
    driver_handler = chros.ChROSDriverInputsHandler(
        driver_inputs_rate, driver, "~/input/driver_inputs"
    )
    ros_manager.RegisterHandler(driver_handler)

    # Handler for publishing vehicle state (position, orientation, etc.)
    vehicle_state_rate = 25  # Hz
    vehicle_state_handler = chros.ChROSBodyHandler(
        vehicle_state_rate,
        hmmwv.GetChassisBody(),
        "~/output/hmmwv/state"
    )
    ros_manager.RegisterHandler(vehicle_state_handler)

    # Finalize the ROS manager (initialize the node and publishers/subscribers)
    ros_manager.Initialize()

    # ------------------------------------------------------------------
    # 6. Simulation loop
    # ------------------------------------------------------------------
    step_size = 1e-3
    sim_time = 0.0
    time_end = 30.0  # seconds

    while sim_time < time_end:
        # Current simulation time
        sim_time = system.GetChTime()

        # --- Collect driver outputs (steering / throttle / braking) ---
        driver_inputs = driver.GetInputs()

        # --- Synchronize subsystems at the current time ---
        driver.Synchronize(sim_time)
        terrain.Synchronize(sim_time)
        hmmwv.Synchronize(sim_time, driver_inputs, terrain)

        # --- Advance the simulation by one timestep ---
        driver.Advance(step_size)
        terrain.Advance(step_size)
        hmmwv.Advance(step_size)

        # --- Update the ROS manager for data publishing/subscribing ---
        if not ros_manager.Update(sim_time, step_size):
            break

    return 0


if __name__ == "__main__":
    main()