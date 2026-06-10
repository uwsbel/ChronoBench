#!/usr/bin/env python3
"""
PyChrono HMMWV + terrain + driver + ROS communication example.

This script creates:
  - An HMMWV full vehicle model
  - Rigid terrain with prescribed friction/restitution
  - A driver system providing steering/throttle/braking inputs
  - A Chrono ROS manager with handlers for:
      * simulation clock
      * driver inputs
      * vehicle state
  - A simulation loop that synchronizes and advances all modules
"""

import os
import tempfile

import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.ros as chros


# ---------------------------------------------------------------------
# User parameters
# ---------------------------------------------------------------------

CONTACT_METHOD = chrono.ChContactMethod_NSC

ENGINE_TYPE = veh.EngineModelType_SHAFTS
TRANSMISSION_TYPE = veh.TransmissionModelType_AUTOMATIC_SHAFTS
DRIVELINE_TYPE = veh.DrivelineTypeWV_AWD
STEERING_TYPE = veh.SteeringTypeWV_PITMAN_ARM
TIRE_MODEL = veh.TireModelType_TMEASY

TERRAIN_FRICTION = 0.9
TERRAIN_RESTITUTION = 0.01

STEP_SIZE = 1.0e-3
TIRE_STEP_SIZE = 1.0e-3
END_TIME = 30.0

ROS_UPDATE_RATE = 50.0

INIT_LOC = chrono.ChVector3d(0.0, 0.0, 0.5)
INIT_ROT = chrono.ChQuaterniond(1.0, 0.0, 0.0, 0.0)


# ---------------------------------------------------------------------
# Utility: construct ROS handlers across minor Chrono API variations
# ---------------------------------------------------------------------

def construct_first(cls, argsets):
    """
    Try several constructor signatures for a Chrono ROS handler.
    This helps with small API differences between Chrono versions.
    """
    last_error = None
    for args in argsets:
        try:
            return cls(*args)
        except TypeError as err:
            last_error = err
    raise last_error


# ---------------------------------------------------------------------
# Initialize Chrono data paths
# ---------------------------------------------------------------------

if "CHRONO_DATA_DIR" in os.environ:
    chrono.SetChronoDataPath(os.environ["CHRONO_DATA_DIR"])

veh.SetDataPath(chrono.GetChronoDataFile("vehicle/"))


# ---------------------------------------------------------------------
# Create and configure the HMMWV vehicle
# ---------------------------------------------------------------------

hmmwv = veh.HMMWV_Full()

hmmwv.SetContactMethod(CONTACT_METHOD)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)

hmmwv.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))

hmmwv.SetEngineType(ENGINE_TYPE)
hmmwv.SetTransmissionType(TRANSMISSION_TYPE)
hmmwv.SetDriveType(DRIVELINE_TYPE)
hmmwv.SetSteeringType(STEERING_TYPE)
hmmwv.SetTireType(TIRE_MODEL)
hmmwv.SetTireStepSize(TIRE_STEP_SIZE)

hmmwv.Initialize()

system = hmmwv.GetSystem()
system.SetGravitationalAcceleration(chrono.ChVector3d(0.0, 0.0, -9.81))

# Optional solver tuning for NSC contact
system.SetNumThreads(1)


# ---------------------------------------------------------------------
# Create rigid terrain
# ---------------------------------------------------------------------

terrain = veh.RigidTerrain(system)

if CONTACT_METHOD == chrono.ChContactMethod_NSC:
    terrain_mat = chrono.ChContactMaterialNSC()
else:
    terrain_mat = chrono.ChContactMaterialSMC()

terrain_mat.SetFriction(TERRAIN_FRICTION)
terrain_mat.SetRestitution(TERRAIN_RESTITUTION)

patch = terrain.AddPatch(
    terrain_mat,
    chrono.ChCoordsysd(
        chrono.ChVector3d(0.0, 0.0, 0.0),
        chrono.ChQuaterniond(1.0, 0.0, 0.0, 0.0),
    ),
    200.0,
    200.0,
)

patch.SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"), 200.0, 200.0)
patch.SetColor(chrono.ChColor(0.45, 0.45, 0.45))

terrain.Initialize()


# ---------------------------------------------------------------------
# Create a basic driver system
# ---------------------------------------------------------------------
# ChDataDriver reads time, steering, throttle, braking from a text file.
# This gives the vehicle a simple open-loop throttle profile. ROS driver
# input handlers can be used to monitor or, depending on the Chrono ROS
# version, override/receive driver input commands.

driver_file = os.path.join(tempfile.gettempdir(), "hmmwv_driver_inputs.txt")

with open(driver_file, "w", encoding="utf-8") as f:
    f.write("# time steering throttle braking\n")
    f.write("0.0   0.0  0.0  0.0\n")
    f.write("0.5   0.0  0.3  0.0\n")
    f.write("5.0   0.0  0.5  0.0\n")
    f.write("10.0  0.15 0.5  0.0\n")
    f.write("15.0 -0.15 0.5  0.0\n")
    f.write("20.0  0.0  0.4  0.0\n")
    f.write("25.0  0.0  0.0  0.3\n")
    f.write("30.0  0.0  0.0  0.5\n")

driver = veh.ChDataDriver(hmmwv.GetVehicle(), driver_file)
driver.Initialize()


# ---------------------------------------------------------------------
# Initialize ROS manager and register handlers
# ---------------------------------------------------------------------

ros_manager = chros.ChROSManager()

# Clock handler publishes simulation time.
clock_handler = construct_first(
    chros.ChROSClockHandler,
    [
        tuple(),
        (system,),
    ],
)
ros_manager.RegisterHandler(clock_handler)

# Driver inputs handler.
# Common signatures are tried for compatibility with different Chrono versions.
driver_inputs_handler = construct_first(
    chros.ChROSDriverInputsHandler,
    [
        (ROS_UPDATE_RATE, driver, "~/input/driver_inputs"),
        (ROS_UPDATE_RATE, driver),
        (driver, "~/input/driver_inputs"),
        (driver,),
    ],
)
ros_manager.RegisterHandler(driver_inputs_handler)

# Vehicle state handler publishes chassis/vehicle state.
vehicle_state_handler = construct_first(
    chros.ChROSVehicleHandler,
    [
        (ROS_UPDATE_RATE, hmmwv.GetVehicle(), "~/output/hmmwv/state"),
        (ROS_UPDATE_RATE, hmmwv.GetVehicle()),
        (hmmwv.GetVehicle(), "~/output/hmmwv/state"),
        (hmmwv.GetVehicle(),),
    ],
)
ros_manager.RegisterHandler(vehicle_state_handler)

ros_manager.Initialize()


# ---------------------------------------------------------------------
# Simulation loop
# ---------------------------------------------------------------------

print("Starting HMMWV PyChrono simulation with ROS communication...")

while system.GetChTime() < END_TIME:
    time = system.GetChTime()

    # Get current driver inputs.
    driver_inputs = driver.GetInputs()

    # Synchronize modules.
    driver.Synchronize(time)
    terrain.Synchronize(time)
    hmmwv.Synchronize(time, driver_inputs, terrain)

    # Update ROS manager.
    # This publishes clock/state data and processes ROS callbacks.
    ros_manager.Update(time, STEP_SIZE)

    # Advance simulation modules.
    driver.Advance(STEP_SIZE)
    terrain.Advance(STEP_SIZE)
    hmmwv.Advance(STEP_SIZE)

print("Simulation complete.")