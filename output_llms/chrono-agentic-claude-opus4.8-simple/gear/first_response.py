import os
import math as m
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemNSC()                                           # rigid-body MBS system
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))         # epicyclic train: ignore gravity (planar gear plane)

# shared visual material with the demo pinkwhite texture for the gear discs
vis_mat = chrono.ChVisualMaterial()                                  # "appropriate materials"
vis_mat.SetKdTexture(chrono.GetChronoDataFile("textures/pinkwhite.png"))  # demo gear texture

radA = 2                                                             # sun gear radius
radB = 4                                                             # planet gear radius
radC = 2 * radB + radA                                               # internal ring radius (concentric with sun)

# fixed truss/wall — acts as the internal ring of the epicyclic set (centered on sun axis)
mbody_truss = chrono.ChBodyEasyBox(20, 10, 2, 1000, True, False)     # decorative wall box
mbody_truss.SetPos(chrono.ChVector3d(0, 0, 3))                       # behind the gear plane
mbody_truss.SetFixed(True)                                           # fixed truss
sys.Add(mbody_truss)

# rotating bar = the carrier arm that holds the planet axle
mbody_train = chrono.ChBodyEasyBox(8, 1.5, 1.0, 1000, True, False)   # carrier bar
mbody_train.SetPos(chrono.ChVector3d(3, 0, 0))                       # spans from sun axis toward the planet
sys.Add(mbody_train)

# carrier revolute to the fixed truss, about world Z (the sun axis)
link_revoluteTT = chrono.ChLinkLockRevolute()                        # carrier <-> truss hinge
link_revoluteTT.Initialize(mbody_train, mbody_truss, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
sys.AddLink(link_revoluteTT)

# sun gear A (central, external teeth) — driven by the gear motor
mbody_gearA = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radA, 0.5, 1000, True, False)  # sun disc
mbody_gearA.SetPos(chrono.ChVector3d(0, 0, -1))                      # gear plane at z = -1
mbody_gearA.SetRot(chrono.QuatFromAngleX(m.pi / 2))                  # lay disc so spin axis is world Z
mbody_gearA.GetVisualShape(0).SetMaterial(0, vis_mat)                # pinkwhite texture
sys.Add(mbody_gearA)
# thin decorative shaft cylinder on the sun so its rotation is visible
mshaft_shape = chrono.ChVisualShapeCylinder(radA * 0.4, 13)          # decorative shaft
mbody_gearA.AddVisualShape(mshaft_shape, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(m.pi / 2)))

# planet gear B (orbits the sun, rides on the carrier)
mbody_gearB = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radB, 0.5, 1000, True, False)  # planet disc
interaxis12 = radA + radB                                            # sun-planet center distance
mbody_gearB.SetPos(chrono.ChVector3d(interaxis12, 0, -1))            # planet on the gear plane
mbody_gearB.SetRot(chrono.QuatFromAngleX(m.pi / 2))                  # lay disc so spin axis is world Z
mbody_gearB.GetVisualShape(0).SetMaterial(0, vis_mat)                # pinkwhite texture
sys.Add(mbody_gearB)

# planet revolute attaches to the CARRIER (not ground), about world Z at the planet center
link_revoluteBchassis = chrono.ChLinkLockRevolute()                  # planet <-> carrier hinge
link_revoluteBchassis.Initialize(mbody_gearB, mbody_train,
                                 chrono.ChFramed(chrono.ChVector3d(interaxis12, 0, 0), chrono.QUNIT))
sys.AddLink(link_revoluteBchassis)

# EXTERNAL mesh: sun A <-> planet B
link_gearAB = chrono.ChLinkLockGear()                                # sun-planet external mesh
link_gearAB.Initialize(mbody_gearA, mbody_gearB, chrono.ChFramed())
link_gearAB.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))
link_gearAB.SetFrameShaft2(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))
link_gearAB.SetTransmissionRatio(radA / radB)                        # external ratio = radA/radB
link_gearAB.SetEnforcePhase(True)                                    # keep teeth phased
sys.AddLink(link_gearAB)

# INTERNAL mesh: planet B <-> ring (the fixed truss), epicyclic
link_gearBC = chrono.ChLinkLockGear()                                # planet-ring internal mesh
link_gearBC.Initialize(mbody_gearB, mbody_truss, chrono.ChFramed())
link_gearBC.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))
link_gearBC.SetFrameShaft2(chrono.ChFramed(chrono.ChVector3d(0, 0, -4), chrono.QUNIT))  # ring shaft behind gear plane
link_gearBC.SetTransmissionRatio(radB / radC)                        # internal ratio = radB/radC
link_gearBC.SetEpicyclic(True)                                       # INTERNAL (ring) mesh
sys.AddLink(link_gearBC)

# gear motor: enforce constant rotation speed of the sun against the truss
link_motor = chrono.ChLinkMotorRotationSpeed()                       # full motor-link, no extra revolute
link_motor.Initialize(mbody_gearA, mbody_truss, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
link_motor.SetSpeedFunction(chrono.ChFunctionConst(6))               # constant 6 rad/s
sys.AddLink(link_motor)

sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_PROJECTED)  # stable gear meshing

# full Irrlicht block (Initialize first, NO grid)
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Epicyclic gears")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(12, 15, -20))                        # look onto the gear plane
vis.AddTypicalLights()                                               # appropriate lighting

time_step = 2e-3                                                     # integration step
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))         # untagged cadence constant
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)
