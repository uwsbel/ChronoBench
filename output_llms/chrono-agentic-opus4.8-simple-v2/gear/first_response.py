import os
import math as m
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemNSC()                                           # NSC rigid-body system
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))     # g = 9.81, Y-up

# shared visual materials for the parts
mat_gear = chrono.ChVisualMaterial()                                 # gear material
mat_gear.SetDiffuseColor(chrono.ChColor(0.2, 0.4, 0.8))             # blue gears
mat_bar = chrono.ChVisualMaterial()                                  # carrier-bar material
mat_bar.SetDiffuseColor(chrono.ChColor(0.9, 0.4, 0.1))             # orange bar

radA = 1.0                                                           # radius of the fixed/sun gear
radB = 0.5                                                           # radius of the orbiting planet gear
interaxis = radA + radB                                              # center distance / carrier length
gear_th = 0.18                                                       # gear disc thickness
bar_z = 0.45                                                         # carrier-bar offset toward camera, clear of disc faces

# the fixed truss (ground) — everything reacts against it
mbody_truss = chrono.ChBodyEasyBox(0.5, 0.5, 0.5, 1000, True, False)  # box marking the truss origin
mbody_truss.SetPos(chrono.ChVector3d(0, 0, -0.5))                    # truss sits behind the gear plane
mbody_truss.SetFixed(True)                                           # truss is fixed to ground
sys.Add(mbody_truss)

# a fixed "sun" gear, rigidly attached to the truss (the gear the planet rolls around)
mbody_gearA = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radA, gear_th, 1000, True, False)  # sun gear disc
mbody_gearA.SetPos(chrono.ChVector3d(0, 0, 0))                       # centered at world origin
mbody_gearA.SetRot(chrono.QuatFromAngleX(m.pi / 2))                  # lay the gear so its axis is +Z
mbody_gearA.GetVisualShape(0).SetMaterial(0, mat_gear)              # color the sun gear
sys.Add(mbody_gearA)

# weld the sun gear to the truss so it never spins (it is the reaction gear)
link_gearA = chrono.ChLinkLockLock()                                # rigid weld sun-gear ↔ truss
link_gearA.Initialize(mbody_gearA, mbody_truss, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
sys.AddLink(link_gearA)

# the rotating bar (carrier arm) that carries the planet gear around the sun
mbody_train = chrono.ChBody()                                        # carrier bar body
mbody_train.SetPos(chrono.ChVector3d(interaxis / 2, 0, bar_z))      # bar spans truss → planet center, offset in +Z
cyl_bar = chrono.ChVisualShapeCylinder(0.12, interaxis)            # thick rod along the arm
mbody_train.AddVisualShape(cyl_bar, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))
mbody_train.GetVisualShape(0).SetMaterial(0, mat_bar)              # color the carrier bar
sys.Add(mbody_train)

# revolute between the carrier bar and the truss — the bar pivots about the sun center (+Z axis)
link_revoluteTB = chrono.ChLinkLockRevolute()                       # carrier ↔ truss hinge
link_revoluteTB.Initialize(mbody_train, mbody_truss, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
sys.AddLink(link_revoluteTB)

# the orbiting planet gear, riding at the far end of the carrier bar
mbody_gearB = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radB, gear_th, 1000, True, False)  # planet gear disc
mbody_gearB.SetPos(chrono.ChVector3d(interaxis, 0, 0))             # at the carrier far end, in the gear plane
mbody_gearB.SetRot(chrono.QuatFromAngleX(m.pi / 2))                 # axis along +Z, same as sun
mbody_gearB.GetVisualShape(0).SetMaterial(0, mat_gear)             # color the planet gear
sys.Add(mbody_gearB)

# revolute between the planet gear and the carrier bar — the planet spins on the bar
link_revoluteBP = chrono.ChLinkLockRevolute()                      # planet ↔ carrier hinge
link_revoluteBP.Initialize(mbody_gearB, mbody_train, chrono.ChFramed(chrono.ChVector3d(interaxis, 0, 0)))
sys.AddLink(link_revoluteBP)

# the epicyclic gear constraint between the fixed sun gear and the orbiting planet gear
link_gearAB = chrono.ChLinkLockGear()                              # gear mesh A ↔ B
link_gearAB.Initialize(mbody_gearA, mbody_gearB, chrono.ChFramed())
link_gearAB.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))  # shaft1 along Z
link_gearAB.SetFrameShaft2(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))  # shaft2 along Z
link_gearAB.SetTransmissionRatio(radA / radB)                     # teeth ratio sun/planet
link_gearAB.SetEpicyclic(True)                                    # internal epicyclic mesh (carrier moves)
sys.AddLink(link_gearAB)

# a gear motor enforcing a CONSTANT rotation speed on the carrier bar against the truss
motor = chrono.ChLinkMotorRotationSpeed()                         # full motor-link (imposes the revolute too)
motor.Initialize(mbody_train, mbody_truss, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
motor.SetSpeedFunction(chrono.ChFunctionConst(6))                # constant 6 rad/s carrier speed
sys.AddLink(motor)

# Irrlicht visualization — Initialize() FIRST, then add scene elements (no grid)
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Epicyclic gears")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1.5, 2.0, 5), chrono.ChVector3d(0.5, 0, 0))   # eye on +Z side so the carrier bar faces the camera
vis.AddTypicalLights()

time_step = 1e-3                                                  # integration step
sim_end = 10.0                                                   # stop time
render_fps = 50.0                                               # review framerate
render_every = max(1, round(1.0 / (render_fps * time_step)))    # untagged cadence constant
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)
        if sys.GetChTime() >= sim_end:
            break
