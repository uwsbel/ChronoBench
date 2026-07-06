"""A CORRECT turn-1 candidate written in a different style: setup helpers, different variable
names, a for-loop over step count, manual CSV assembly. Same physics (Polaris tire, CRM soil,
2500 N, 0.2 m/s carriage, 10 RPM, 5 s), so it must score ~100."""
import json

import pychrono as ch
import pychrono.vehicle as vehicle

NORMAL_LOAD = 2500.0
WHEEL_RPM = 10.0
DT = 2e-4
DURATION = 5.0
N_STEPS = int(round(DURATION / DT))


def build_rig():
    whl = vehicle.ReadWheelJSON(vehicle.GetVehicleDataFile("Polaris/Polaris_Wheel.json"))
    tr = vehicle.ReadTireJSON(vehicle.GetVehicleDataFile("Polaris/Polaris_RigidMeshTire.json"))
    system = ch.ChSystemNSC()
    system.SetCollisionSystemType(ch.ChCollisionSystem.Type_BULLET)
    system.SetSolverType(ch.ChSolver.Type_BARZILAIBORWEIN)
    tr.SetStepsize(DT)
    r = vehicle.ChTireTestRig(whl, tr, system)
    r.SetGravitationalAcceleration(9.8)
    r.SetNormalLoad(NORMAL_LOAD)
    r.SetTireStepsize(DT)
    r.SetTireVisualizationType(ch.VisualizationType_NONE)
    return r, system


def attach_crm_terrain(r):
    patch = vehicle.TerrainPatchSize()
    patch.length, patch.width, patch.depth = 4.0, 0.8, 0.2
    soil = vehicle.TerrainParamsCRM()
    soil.sph_params.initial_spacing = 0.02
    soil.mat_props.density = 1700
    soil.mat_props.Young_modulus = 2e6
    soil.mat_props.cohesion_coeff = 1e2
    r.SetTerrainCRM(patch, soil)


rig, sysNSC = build_rig()
attach_crm_terrain(rig)
rig.SetLongSpeedFunction(ch.ChFunctionConst(0.2))
rig.SetAngSpeedFunction(ch.ChFunctionConst(WHEEL_RPM * ch.CH_RPM_TO_RAD_S))
rig.SetTimeDelay(1.0)
rig.Initialize(vehicle.ChTireTestRig.Mode_TEST, 0.05)

hub = rig.GetSpindle()

samples = []
lines = ["t,z,dbp,slip"]
for k in range(1, N_STEPS + 1):
    tk = sysNSC.GetChTime()
    rig.Advance(DT)
    if k % 10 == 0:
        rec = (tk, hub.GetPos().z, rig.GetDBP(), rig.GetLongitudinalSlip())
        samples.append(rec)
        lines.append(f"{rec[0]:.6f},{rec[1]:.6e},{rec[2]:.6e},{rec[3]:.6e}")

with open("out.csv", "w") as fh:
    fh.write("\n".join(lines) + "\n")

late = [rec for rec in samples if rec[0] >= DURATION - 1.0]
m = len(late)
print(json.dumps({"z_settled": sum(rec[1] for rec in late) / m,
                  "dbp_tail": sum(rec[2] for rec in late) / m,
                  "slip_tail": sum(rec[3] for rec in late) / m,
                  "load": NORMAL_LOAD}))
