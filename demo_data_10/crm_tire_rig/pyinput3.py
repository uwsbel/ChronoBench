"""Single tire on CRM deformable terrain (tire test rig), turn 2 (MODIFY) -- PyChrono 10.0,
HIP build, headless -- contracted reference.

Same Polaris tire and CRM soil as turn 1, but the normal load DOUBLES: 2500 -> 5000 N. The
terramechanics law being probed: a heavier wheel sinks deeper into granular terrain, so the
settled spindle height must drop below the turn-1 band (an unmodified candidate fails high).
Everything else (soil, speeds, rig sequencing, outputs) is unchanged; tail means over the last
second are what the judge grades.
"""
import csv
import json

import pychrono as chrono
import pychrono.vehicle as veh

LOAD = 5000.0
RPM = 10.0
STEP = 2e-4
T_DELAY = 1.0
T_END = 5.0

wheel = veh.ReadWheelJSON(veh.GetVehicleDataFile("Polaris/Polaris_Wheel.json"))
tire = veh.ReadTireJSON(veh.GetVehicleDataFile("Polaris/Polaris_RigidMeshTire.json"))

sys = chrono.ChSystemNSC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
sys.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
tire.SetStepsize(STEP)

rig = veh.ChTireTestRig(wheel, tire, sys)
rig.SetGravitationalAcceleration(9.8)
rig.SetNormalLoad(LOAD)
rig.SetTireStepsize(STEP)
rig.SetTireVisualizationType(chrono.VisualizationType_NONE)

size = veh.TerrainPatchSize()
size.length = 4.0
size.width = 0.8
size.depth = 0.2

params = veh.TerrainParamsCRM()
params.sph_params.initial_spacing = 0.02
params.mat_props.density = 1700
params.mat_props.Young_modulus = 2e6
params.mat_props.cohesion_coeff = 1e2
rig.SetTerrainCRM(size, params)

rig.SetLongSpeedFunction(chrono.ChFunctionConst(0.2))
rig.SetAngSpeedFunction(chrono.ChFunctionConst(RPM * chrono.CH_RPM_TO_RAD_S))
rig.SetTimeDelay(T_DELAY)
rig.Initialize(veh.ChTireTestRig.Mode_TEST, 0.05)

spindle = rig.GetSpindle()

t = 0.0
rows = []
n = 0
while t < T_END:
    t = sys.GetChTime()
    rig.Advance(STEP)
    n += 1
    if n % 10 == 0:                      # sample every 2e-3 s
        rows.append((t, spindle.GetPos().z, rig.GetDBP(), rig.GetLongitudinalSlip()))

with open("out.csv", "w", newline="") as fh:
    w = csv.writer(fh)
    w.writerow(["t", "z", "dbp", "slip"])
    for r in rows:
        w.writerow([f"{r[0]:.6f}", f"{r[1]:.6e}", f"{r[2]:.6e}", f"{r[3]:.6e}"])

tail = [r for r in rows if r[0] >= T_END - 1.0]
n = len(tail)
print(json.dumps({"z_settled": sum(r[1] for r in tail) / n,
                  "dbp_tail": sum(r[2] for r in tail) / n,
                  "slip_tail": sum(r[3] for r in tail) / n,
                  "load": LOAD}))
