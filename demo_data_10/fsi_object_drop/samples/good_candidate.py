"""A CORRECT turn-1 candidate written in a different style: helper function for the SPH setup,
different variable names, while-loop over a step counter, manual CSV lines. Same physics (sphere
R = 0.12, density 500, 0.8 x 0.8 x 0.5 tank, spacing 0.025, 3 s), so it must score ~100."""
import json

import pychrono as ch
import pychrono.fsi as chfsi

RADIUS = 0.12
RHO_SPHERE = 500.0
TANK = ch.ChVector3d(0.8, 0.8, 0.5)
H = 0.025
DT = 1e-4
DT_META = 5e-4
DURATION = 3.0


def make_sph_params():
    p = chfsi.SPHParameters()
    p.integration_scheme = chfsi.IntegrationScheme_RK2
    p.num_bce_layers = 4
    p.initial_spacing = H
    p.d0_multiplier = 1
    p.max_velocity = 4.0
    p.shifting_method = chfsi.ShiftingMethod_XSPH
    p.shifting_xsph_eps = 0.5
    p.artificial_viscosity = 0.03
    p.eos_type = chfsi.EosType_TAIT
    p.viscosity_method = chfsi.ViscosityMethod_ARTIFICIAL_UNILATERAL
    p.boundary_method = chfsi.BoundaryMethod_ADAMI
    p.num_proximity_search_steps = 1
    p.use_delta_sph = True
    p.delta_sph_coefficient = 0.1
    p.use_variable_time_step = True
    return p


mbs = ch.ChSystemNSC()
mbs.SetCollisionSystemType(ch.ChCollisionSystem.Type_BULLET)

problem = chfsi.ChFsiProblemCartesian(H, mbs)
problem.SetVerbose(False)
problem.SetGravitationalAcceleration(ch.ChVector3d(0, 0, -9.8))
problem.SetStepSizeCFD(DT)
problem.SetStepsizeMBD(DT)

water = chfsi.FluidProperties()
water.density = 1000.0
water.viscosity = 1.0
problem.SetCfdSPH(water)
problem.SetSPHParameters(make_sph_params())

shape = ch.ChSphere(RADIUS)
m = RHO_SPHERE * shape.GetVolume()
geom = ch.ChBodyGeometry()
geom.materials.append(ch.ChContactMaterialData())
geom.coll_spheres.append(ch.SphereShape(ch.VNULL, shape, 0))

ball = ch.ChBody()
ball.SetName("object")
ball.SetPos(ch.ChVector3d(0, 0, 1.05 * TANK.z + RADIUS))
ball.SetMass(m)
ball.SetInertia(m * shape.GetGyration())
mbs.AddBody(ball)

problem.AddRigidBody(ball, geom, True, True)
problem.RegisterParticlePropertiesCallback(chfsi.DepthPressurePropertiesCallback(TANK.z))
problem.Construct(TANK, ch.ChVector3d(0, 0, 0), chfsi.BoxSide_ALL & ~chfsi.BoxSide_Z_POS)
problem.Initialize()

lines = ["t,z,zrel"]
history = []
n = 0
while n * DT_META < DURATION:
    problem.DoStepDynamics(DT_META)
    n += 1
    tk = n * DT_META
    zk = ball.GetPos().z
    history.append((tk, zk - TANK.z))
    lines.append(f"{tk:.6f},{zk:.6e},{zk - TANK.z:.6e}")

with open("out.csv", "w") as fh:
    fh.write("\n".join(lines) + "\n")

settle = [zr for tk, zr in history if tk >= DURATION - 1.0]
print(json.dumps({"z_settled_rel_surface": sum(settle) / len(settle),
                  "density": RHO_SPHERE, "behavior": "floats"}))
