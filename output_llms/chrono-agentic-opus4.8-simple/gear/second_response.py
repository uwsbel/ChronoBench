import os
import math as m
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemNSC()                                           # NSC system for the gear train

mat = chrono.ChContactMaterialNSC()                                  # shared contact material for the easy bodies

radA = 1.5                                                           # pitch radius of gear A (driver)
radB = 3.5                                                           # pitch radius of gear B (driven)

# ...the truss (fixed support that carries the shafts)
mbody_truss = chrono.ChBodyEasyBox(15, 8, 2, 1000, True, False, mat) # supporting truss box
sys.Add(mbody_truss)
mbody_truss.SetFixed(True)                                           # the truss is the ground reference
mbody_truss.SetPos(chrono.ChVector3d(0, 0, 3))                      # placed behind the gear plane

# shared visualization material for the gears
vis_mat = chrono.ChVisualMaterial()
vis_mat.SetKdTexture(chrono.GetChronoDataFile('textures/pinkwhite.png'))

# ...the first gear (driver)
mbody_gearA = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radA, 0.5, 1000, True, False, mat)  # gear A disk
sys.Add(mbody_gearA)
mbody_gearA.SetPos(chrono.ChVector3d(0, 0, -1))                     # gear A on the front gear plane
mbody_gearA.SetRot(chrono.QuatFromAngleX(m.pi / 2))                # lay the disk so its axis points along Z
mbody_gearA.GetVisualShape(0).SetMaterial(0, vis_mat)

# thin cylinder added only as a visualization of the shaft
mshaft_shape = chrono.ChVisualShapeCylinder(radA * 0.3, 10)         # thinner, shorter visual shaft
mbody_gearA.AddVisualShape(mshaft_shape,
                           chrono.ChFramed(chrono.ChVector3d(0, 3.5, 0),
                                           chrono.QuatFromAngleX(chrono.CH_PI_2)))  # along the gear axis

# ...impose rotation speed between the first gear and the fixed truss
link_motor = chrono.ChLinkMotorRotationSpeed()                      # speed-controlled motor (full motor-link)
link_motor.Initialize(mbody_gearA, mbody_truss,
                      chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))  # at gear A axis
link_motor.SetSpeedFunction(chrono.ChFunctionConst(3))             # constant 3 rad/s drive speed
sys.AddLink(link_motor)

# ...the second gear (driven)
interaxis12 = radA + radB                                          # center distance between the two axes
mbody_gearB = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radB, 0.4, 1000, True, False, mat)  # gear B disk
sys.Add(mbody_gearB)
mbody_gearB.SetPos(chrono.ChVector3d(interaxis12, 0, -2))          # gear B offset along X by center distance
mbody_gearB.SetRot(chrono.QuatFromAngleX(m.pi / 2))               # axis along Z like gear A
mbody_gearB.GetVisualShape(0).SetMaterial(0, vis_mat)

# ...the second gear is hinged to the fixed truss by a revolute about Z
link_revolute = chrono.ChLinkLockRevolute()                        # gear B hinge to the truss
link_revolute.Initialize(mbody_gearB, mbody_truss,
                         chrono.ChFramed(chrono.ChVector3d(interaxis12, 0, 0), chrono.QUNIT))
sys.AddLink(link_revolute)

# ...the gear constraint between wheels A and B (imposes the transmission ratio)
link_gearAB = chrono.ChLinkLockGear()                              # gear pair constraint
link_gearAB.Initialize(mbody_gearA, mbody_gearB, chrono.ChFramed())
link_gearAB.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))  # gear A shaft = Z
link_gearAB.SetFrameShaft2(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))  # gear B shaft = Z
link_gearAB.SetTransmissionRatio(radA / radB)                     # transmission ratio = radA/radB
link_gearAB.SetEnforcePhase(True)                                 # keep the teeth phased
sys.AddLink(link_gearAB)

# Create the Irrlicht visualization (Initialize first, then scene elements; NO grid)
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Gears')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(12, 15, -20))                     # eye position looking at the gear plane
vis.AddTypicalLights()

# projected implicit Euler keeps the gear constraint stable
sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_PROJECTED)

time_step = 1e-3                                                   # integration step
sim_end = 10.0                                                     # total simulated time
render_fps = 50.0                                                  # review frame rate
render_every = max(1, round(1.0 / (render_fps * time_step)))      # untagged cadence constant
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)
        if sys.GetChTime() >= sim_end:
            break
