"""A CORRECT-BUT-DIFFERENT turn-1 HMMWV run: same physics, different style. Uses a custom
veh.ChDriver subclass (the shipped-demo idiom) instead of setting inputs inline, tracks the
starting position before the loop, and logs through a list of dicts. Should pass L1/L2/L3 near
ceiling."""
import csv
import json

import pychrono as chrono
import pychrono.vehicle as veh

T_END, STEP = 4.0, 2e-3


class RampDriver(veh.ChDriver):
    def __init__(self, vehicle):
        veh.ChDriver.__init__(self, vehicle)

    def Synchronize(self, time):
        self.SetThrottle(min(0.7, 3.5 * max(0.0, time - 0.2)))
        self.SetSteering(0.0)
        self.SetBraking(0.0)


car = veh.HMMWV_Full()
car.SetContactMethod(chrono.ChContactMethod_SMC)
car.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(-6, 0, 0.6), chrono.ChQuaterniond(1, 0, 0, 0)))
car.SetEngineType(veh.EngineModelType_SHAFTS)
car.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
car.SetDriveType(veh.DrivelineTypeWV_AWD)
car.SetTireType(veh.TireModelType_RIGID)
car.Initialize()
car.SetChassisVisualizationType(chrono.VisualizationType_NONE)
car.SetWheelVisualizationType(chrono.VisualizationType_NONE)
car.SetTireVisualizationType(chrono.VisualizationType_NONE)
system = car.GetSystem()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

ground = veh.RigidTerrain(system)
surface = chrono.ChContactMaterialSMC()
surface.SetFriction(0.9)
ground.AddPatch(surface, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 40.0, 8.0)
ground.Initialize()

pilot = RampDriver(car.GetVehicle())
pilot.Initialize()

rear_left = car.GetVehicle().GetWheel(1, veh.LEFT)
x_start = car.GetVehicle().GetPos().x

log = []
while system.GetChTime() < T_END:
    t = system.GetChTime()
    inputs = pilot.GetInputs()
    pilot.Synchronize(t)
    ground.Synchronize(t)
    car.Synchronize(t, inputs, ground)
    pilot.Advance(STEP)
    ground.Advance(STEP)
    car.Advance(STEP)
    wp = rear_left.GetSpindle().GetPos()
    log.append({"t": t,
                "dist": car.GetVehicle().GetPos().x - x_start,
                "speed": car.GetVehicle().GetSpeed(),
                "rut": -ground.GetHeight(chrono.ChVector3d(wp.x, wp.y, 0))})

with open("out.csv", "w", newline="") as fh:
    w = csv.DictWriter(fh, fieldnames=["t", "dist", "speed", "rut"])
    w.writeheader()
    for row in log:
        w.writerow({k: f"{v:.6f}" if k == "t" else f"{v:.6e}" for k, v in row.items()})

print(json.dumps({"dist": log[-1]["dist"], "final_speed": log[-1]["speed"],
                  "rut_max": max(r["rut"] for r in log), "terrain": "rigid"}))
