"""Sphere dropped into a water tank (FSI-SPH), turn 1 (CREATE) -- PyChrono 10.0, HIP build,
headless -- contracted reference.

A rigid sphere (R = 0.12 m, density 500 = half of water) is released just above a 0.8 x 0.8 x 0.5
water tank (ChFsiProblemCartesian, spacing 0.025) and settles floating. Archimedes for a
half-density sphere puts the settled CENTER exactly AT the free surface; on this build the SPH
BCE skin adds ~half a spacing of effective radius, so the measured center sits slightly high
(calibrated in contract.json / CONTRACT.md). Logs per meta step: the sphere center height z and
its offset from the still-water surface. The judge derives the settled value from the CSV tail.
"""
import csv
import json

import pychrono as chrono
import pychrono.fsi as fsi

DENSITY = 500.0
R = 0.12
FSIZE = chrono.ChVector3d(0.8, 0.8, 0.5)
SPACING = 0.025
STEP = 1e-4
META = 5 * STEP
T_END = 3.0

sysMBS = chrono.ChSystemNSC()
sysMBS.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

prob = fsi.ChFsiProblemCartesian(SPACING, sysMBS)
prob.SetVerbose(False)
prob.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.8))
prob.SetStepSizeCFD(STEP)
prob.SetStepsizeMBD(STEP)

props = fsi.FluidProperties()
props.density = 1000.0
props.viscosity = 1.0
prob.SetCfdSPH(props)

sph = fsi.SPHParameters()
sph.integration_scheme = fsi.IntegrationScheme_RK2
sph.num_bce_layers = 4
sph.initial_spacing = SPACING
sph.d0_multiplier = 1
sph.max_velocity = 4.0
sph.shifting_method = fsi.ShiftingMethod_XSPH
sph.shifting_xsph_eps = 0.5
sph.artificial_viscosity = 0.03
sph.eos_type = fsi.EosType_TAIT
sph.viscosity_method = fsi.ViscosityMethod_ARTIFICIAL_UNILATERAL
sph.boundary_method = fsi.BoundaryMethod_ADAMI
sph.num_proximity_search_steps = 1
sph.use_delta_sph = True
sph.delta_sph_coefficient = 0.1
sph.use_variable_time_step = True
prob.SetSPHParameters(sph)

sphere = chrono.ChSphere(R)
mass = DENSITY * sphere.GetVolume()
geometry = chrono.ChBodyGeometry()
geometry.materials.append(chrono.ChContactMaterialData())
geometry.coll_spheres.append(chrono.SphereShape(chrono.VNULL, sphere, 0))

body = chrono.ChBody()
body.SetName("object")
body.SetPos(chrono.ChVector3d(0, 0, 1.05 * FSIZE.z + R))
body.SetMass(mass)
body.SetInertia(mass * sphere.GetGyration())
sysMBS.AddBody(body)

prob.AddRigidBody(body, geometry, True, True)
prob.RegisterParticlePropertiesCallback(fsi.DepthPressurePropertiesCallback(FSIZE.z))
prob.Construct(FSIZE, chrono.ChVector3d(0, 0, 0), fsi.BoxSide_ALL & ~fsi.BoxSide_Z_POS)
prob.Initialize()

t = 0.0
rows = []
while t < T_END:
    prob.DoStepDynamics(META)
    t += META
    z = body.GetPos().z
    rows.append((t, z, z - FSIZE.z))

with open("out.csv", "w", newline="") as fh:
    w = csv.writer(fh)
    w.writerow(["t", "z", "zrel"])
    for r in rows:
        w.writerow([f"{r[0]:.6f}", f"{r[1]:.6e}", f"{r[2]:.6e}"])

tail = [r[2] for r in rows if r[0] >= T_END - 1.0]
print(json.dumps({"z_settled_rel_surface": sum(tail) / len(tail),
                  "density": DENSITY, "behavior": "floats"}))
