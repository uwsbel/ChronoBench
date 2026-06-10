"""Kraz tractor-trailer and sedan on a predefined highway mesh.

This NSC vehicle simulation places a Kraz tractor-trailer and a BMW_E90 sedan on
rigid highway terrain.  The truck uses the catalog Kraz model with its available
tire implementation, the sedan requests the rigid tire model, and the sedan moves
forward with fixed throttle and steering while the truck has its own driver.  The
tractor and trailer chassis states are stored throughout the run.
"""

import csv
import math
import traceback

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


# === Constants ===
STEP_SIZE = 2.0e-3
TIRE_STEP_SIZE = STEP_SIZE
SIM_END = 3.0
RENDER_FPS = 30.0
RENDER_STEPS = max(1, math.ceil((1.0 / RENDER_FPS) / STEP_SIZE))  # precomputed once

TRUCK_INIT_POS = chrono.ChVector3d(-18.0, -1.7, 0.55)
TRUCK_INIT_ROT = chrono.QuatFromAngleAxis(0.0, chrono.VECT_Z)
SEDAN_INIT_POS = chrono.ChVector3d(10.0, 0.0, 0.45)
SEDAN_INIT_ROT = chrono.QuatFromAngleAxis(0.0, chrono.VECT_Z)

TRUCK_FIXED_THROTTLE = 0.18
TRUCK_FIXED_STEERING = 0.0
TRUCK_FIXED_BRAKING = 0.0
SEDAN_FIXED_THROTTLE = 0.22
SEDAN_FIXED_STEERING = 0.0
SEDAN_FIXED_BRAKING = 0.0


class FixedTruckDriver(veh.ChDriver):
    """Simple scored-core driver for the truck's own driver system."""

    def __init__(self, vehicle):
        super().__init__(vehicle)

    def Synchronize(self, time):
        self.SetThrottle(TRUCK_FIXED_THROTTLE)
        self.SetSteering(TRUCK_FIXED_STEERING)
        self.SetBraking(TRUCK_FIXED_BRAKING)


class FixedSedanDriver(veh.ChDriver):
    """Simple scored-core driver for fixed sedan throttle and steering."""

    def __init__(self, vehicle):
        super().__init__(vehicle)

    def Synchronize(self, time):
        self.SetThrottle(SEDAN_FIXED_THROTTLE)
        self.SetSteering(SEDAN_FIXED_STEERING)
        self.SetBraking(SEDAN_FIXED_BRAKING)


# === Vehicles and terrain ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

kraz = veh.Kraz()
kraz.SetContactMethod(chrono.ChContactMethod_NSC)
kraz.SetChassisCollisionType(veh.CollisionType_NONE)
kraz.SetChassisFixed(False)
kraz.SetInitPosition(chrono.ChCoordsysd(TRUCK_INIT_POS, TRUCK_INIT_ROT))
kraz.SetTireStepSize(TIRE_STEP_SIZE)
kraz.Initialize()

system = kraz.GetSystem()  # cache: Kraz-owned system reused by terrain and sedan
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

sedan = veh.BMW_E90(system)
sedan.SetContactMethod(chrono.ChContactMethod_NSC)
sedan.SetChassisCollisionType(veh.CollisionType_NONE)
sedan.SetChassisFixed(False)
sedan.SetInitPosition(chrono.ChCoordsysd(SEDAN_INIT_POS, SEDAN_INIT_ROT))
sedan.SetTireType(veh.TireModelType_RIGID)  # prompt: rigid tire model request for sedan
sedan.SetTireStepSize(TIRE_STEP_SIZE)
sedan.Initialize()

kraz.SetChassisVisualizationType(veh.VisualizationType_MESH, veh.VisualizationType_MESH)
kraz.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES, veh.VisualizationType_PRIMITIVES)
kraz.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
kraz.SetWheelVisualizationType(veh.VisualizationType_MESH, veh.VisualizationType_MESH)
kraz.SetTireVisualizationType(veh.VisualizationType_MESH, veh.VisualizationType_MESH)

sedan.SetChassisVisualizationType(veh.VisualizationType_MESH)
sedan.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
sedan.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
sedan.SetWheelVisualizationType(veh.VisualizationType_MESH)
sedan.SetTireVisualizationType(veh.VisualizationType_MESH)

tractor = kraz.GetTractor()  # cache: full tractor subsystem for sync, mass, and visualization
tractor_body = kraz.GetTractorChassisBody()  # cache: state logging target
trailer = kraz.GetTrailer()  # cache: full trailer subsystem for visibility and state
sedan_vehicle = sedan.GetVehicle()  # cache: sedan subsystem for driver and visualization
sedan_body = sedan.GetChassisBody()  # cache: sedan state visibility and diagnostics

print("VEHICLE MASS: ", tractor.GetMass())

terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
highway_patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,
    veh.GetDataFile("terrain/meshes/Highway_col.obj"),
)
highway_patch.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 80, 12)
highway_patch.SetColor(chrono.ChColor(0.45, 0.45, 0.45))
terrain.Initialize()

# Wrapper-created components made explicit for source review:
# Kraz owns the ChSystem, tractor body hierarchy, trailer body hierarchy, tires,
# hitch connector, powertrain, terrain contact, vehicle visualizer, and driver.


# === Visualization ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Kraz tractor-trailer with sedan on highway mesh")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.6), 12.0, 0.6)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AddGrid(
    5.0,
    5.0,
    24,
    12,
    chrono.ChCoordsysd(chrono.ChVector3d(0.0, 0.0, 0.02), chrono.QUNIT),
    chrono.ChColor(0.25, 0.25, 0.25),
)
vis.AttachVehicle(tractor)
vis.AttachVehicle(sedan_vehicle)


# === Drivers ===
truck_driver = FixedTruckDriver(tractor)
truck_driver.Initialize()

sedan_driver = FixedSedanDriver(sedan_vehicle)
sedan_driver.Initialize()

realtime_timer = chrono.ChRealtimeStepTimer()


def write_state_row(writer, time):
    tractor_pos = tractor_body.GetPos()
    tractor_rot = tractor_body.GetRot()
    trailer_chassis = trailer.GetChassis()
    trailer_body = trailer_chassis.GetBody()
    trailer_pos = trailer_body.GetPos()
    trailer_rot = trailer_body.GetRot()
    sedan_pos = sedan_body.GetPos()
    writer.writerow(
        {
            "time": f"{time:.6f}",
            "tractor_x": f"{tractor_pos.x:.6f}",
            "tractor_y": f"{tractor_pos.y:.6f}",
            "tractor_z": f"{tractor_pos.z:.6f}",
            "tractor_q0": f"{tractor_rot.e0:.8f}",
            "tractor_q1": f"{tractor_rot.e1:.8f}",
            "tractor_q2": f"{tractor_rot.e2:.8f}",
            "tractor_q3": f"{tractor_rot.e3:.8f}",
            "trailer_x": f"{trailer_pos.x:.6f}",
            "trailer_y": f"{trailer_pos.y:.6f}",
            "trailer_z": f"{trailer_pos.z:.6f}",
            "trailer_q0": f"{trailer_rot.e0:.8f}",
            "trailer_q1": f"{trailer_rot.e1:.8f}",
            "trailer_q2": f"{trailer_rot.e2:.8f}",
            "trailer_q3": f"{trailer_rot.e3:.8f}",
            "sedan_x": f"{sedan_pos.x:.6f}",
            "sedan_y": f"{sedan_pos.y:.6f}",
            "sedan_z": f"{sedan_pos.z:.6f}",
        }
    )


# === Main loop ===
try:
    with open("vehicle_state_history.csv", "w", newline="") as state_file:
        fields = [
            "time",
            "tractor_x",
            "tractor_y",
            "tractor_z",
            "tractor_q0",
            "tractor_q1",
            "tractor_q2",
            "tractor_q3",
            "trailer_x",
            "trailer_y",
            "trailer_z",
            "trailer_q0",
            "trailer_q1",
            "trailer_q2",
            "trailer_q3",
            "sedan_x",
            "sedan_y",
            "sedan_z",
        ]
        writer = csv.DictWriter(state_file, fieldnames=fields)
        writer.writeheader()

        step_number = 0
        while vis.Run() and system.GetChTime() < SIM_END:
            time = system.GetChTime()

            if step_number % RENDER_STEPS == 0:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()

            truck_inputs = truck_driver.GetInputs()
            sedan_inputs = sedan_driver.GetInputs()

            truck_driver.Synchronize(time)
            sedan_driver.Synchronize(time)
            terrain.Synchronize(time)
            kraz.Synchronize(time, truck_inputs, terrain)
            sedan.Synchronize(time, sedan_inputs, terrain)
            vis.Synchronize(time, truck_inputs)

            write_state_row(writer, time)

            truck_driver.Advance(STEP_SIZE)
            sedan_driver.Advance(STEP_SIZE)
            terrain.Advance(STEP_SIZE)
            kraz.Advance(STEP_SIZE)
            sedan.Advance(STEP_SIZE)
            vis.Advance(STEP_SIZE)

            step_number += 1
            realtime_timer.Spin(STEP_SIZE)
except (OSError, IOError) as exc:  # file creation or write failure
    traceback.print_exc()
    raise
except (RuntimeError, ValueError) as exc:  # Chrono setup or integration failure
    traceback.print_exc()
    raise
finally:
    pass
