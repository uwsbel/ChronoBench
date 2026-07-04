"""A RUNS-BUT-WRONG turn-1 HMMWV run: structurally complete and it executes cleanly (passes L1 and
the minimal L2 capability checks, SetThrottle appears in the source), but the throttle cap has a
typo, 0.07 instead of 0.7, so the vehicle crawls: distance and final speed land far below the
calibrated bands. The CSV-derived L3 invariants catch it and the wrong-physics cap applies."""
import csv
import json

import pychrono as chrono
import pychrono.vehicle as veh

T_END, STEP = 4.0, 2e-3

hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)
hmmwv.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(-6, 0, 0.6), chrono.ChQuaterniond(1, 0, 0, 0)))
hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
hmmwv.SetTireType(veh.TireModelType_RIGID)
hmmwv.Initialize()
hmmwv.SetChassisVisualizationType(chrono.VisualizationType_NONE)
hmmwv.SetWheelVisualizationType(chrono.VisualizationType_NONE)
hmmwv.SetTireVisualizationType(chrono.VisualizationType_NONE)
sys = hmmwv.GetSystem()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

terrain = veh.RigidTerrain(sys)
mat = chrono.ChContactMaterialSMC()
mat.SetFriction(0.9)
terrain.AddPatch(mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 40.0, 8.0)
terrain.Initialize()

driver = veh.ChDriver(hmmwv.GetVehicle())
driver.Initialize()

rear_left = hmmwv.GetVehicle().GetWheel(1, veh.LEFT)   # axle 1 = rear

x0 = None
rows = []
t = 0.0
while t < T_END:
    t = sys.GetChTime()
    driver.SetThrottle(min(0.07, 3.5 * max(0.0, t - 0.2)))   # WRONG: throttle capped at 0.07 (typo)
    driver.SetSteering(0.0)
    driver.SetBraking(0.0)
    inputs = driver.GetInputs()
    driver.Synchronize(t)
    terrain.Synchronize(t)
    hmmwv.Synchronize(t, inputs, terrain)
    driver.Advance(STEP)
    terrain.Advance(STEP)
    hmmwv.Advance(STEP)
    pos = hmmwv.GetVehicle().GetPos()
    if x0 is None:
        x0 = pos.x
    wp = rear_left.GetSpindle().GetPos()
    rut = -terrain.GetHeight(chrono.ChVector3d(wp.x, wp.y, 0))
    rows.append((t, pos.x - x0, hmmwv.GetVehicle().GetSpeed(), rut))

with open("out.csv", "w", newline="") as fh:
    w = csv.writer(fh)
    w.writerow(["t", "dist", "speed", "rut"])
    for r in rows:
        w.writerow([f"{r[0]:.6f}", f"{r[1]:.6e}", f"{r[2]:.6e}", f"{r[3]:.6e}"])

print(json.dumps({"dist": rows[-1][1], "final_speed": rows[-1][2],
                  "rut_max": max(r[3] for r in rows), "terrain": "rigid"}))
