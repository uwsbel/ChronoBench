import os
import math as m
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# gear train demo — radA=1.5, radB=3.5, truss 15×8×2, motor speed 3 rad/s
# gear B positioned at z=-2, visual shaft uses radA*0.3 radius, length 10

sys = chrono.ChSystemNSC()                                           # NSC for pure jointed MBS
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))    # Y-up, g=9.81

# contact material for truss (used by ChBodyEasyBox with vis+no-collide)
mat = chrono.ChContactMaterialNSC()                                  # NSC material

# gear radii and inter-axis distance
radA = 1.5                                                           # driving gear radius
radB = 3.5                                                           # driven gear radius
interaxis12 = radA + radB                                            # distance between gear axes

# ---- truss (fixed support) ----
mbody_truss = chrono.ChBodyEasyBox(15, 8, 2, 1000, True, False, mat)  # 15×8×2, vis=True, collide=False
mbody_truss.SetFixed(True)                                           # fixed to world
mbody_truss.SetPos(chrono.ChVector3d(0, 0, 3))                       # truss center
sys.AddBody(mbody_truss)

# ---- gear A (driving gear) ----
mbody_gearA = chrono.ChBodyEasyCylinder(chrono.ChAxis_Z, radA, 0.5, 700, True, False, mat)  # r=1.5, h=0.5
mbody_gearA.SetPos(chrono.ChVector3d(0, 0, -1))                      # gear A center position
mbody_gearA.SetRot(chrono.QUNIT)                                     # upright in XY plane
sys.AddBody(mbody_gearA)

# visual shaft for gear A — thin cylinder along Z
mshaft_shape = chrono.ChVisualShapeCylinder(radA * 0.3, 10)          # r=0.45, length=10
mbody_gearA.AddVisualShape(mshaft_shape, chrono.ChFramed(chrono.VNULL, chrono.QUNIT))  # along Z axis

# ---- gear B (driven gear) ----
mbody_gearB = chrono.ChBodyEasyCylinder(chrono.ChAxis_Z, radB, 0.5, 700, True, False, mat)  # r=3.5, h=0.5
mbody_gearB.SetPos(chrono.ChVector3d(interaxis12, 0, -2))            # gear B at (interaxis, 0, -2)
mbody_gearB.SetRot(chrono.QUNIT)
sys.AddBody(mbody_gearB)

# ---- motor: drives gear A against truss ----
link_motor = chrono.ChLinkMotorRotationSpeed()                       # full motor-link, no extra revolute
link_motor.Initialize(
    mbody_gearA, mbody_truss,
    chrono.ChFramed(chrono.ChVector3d(0, 0, -1),
                    chrono.QuatFromAngleX(-m.pi / 2))                # shaft along Z
)
link_motor.SetSpeedFunction(chrono.ChFunctionConst(3))               # 3 rad/s driving speed
sys.AddLink(link_motor)

# ---- revolute: gear B pivots on truss ----
link_revolute_B = chrono.ChLinkLockRevolute()                        # hinge for gear B
link_revolute_B.Initialize(
    mbody_gearB, mbody_truss,
    chrono.ChFramed(chrono.ChVector3d(interaxis12, 0, -2),
                    chrono.QuatFromAngleX(-m.pi / 2))                # hinge axis = Z
)
sys.AddLink(link_revolute_B)

# ---- gear constraint between A and B ----
link_gear = chrono.ChLinkLockGear()                                  # gear constraint
link_gear.Initialize(mbody_gearA, mbody_gearB, chrono.ChFramed())
link_gear.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))
link_gear.SetFrameShaft2(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))
link_gear.SetTransmissionRatio(radA / radB)                          # ratio = 1.5/3.5
link_gear.SetEnforcePhase(True)                                      # keep teeth phased
sys.AddLink(link_gear)

# ---- Irrlicht visualization ----
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Gear Train - radA=1.5 radB=3.5")
vis.Initialize()                                                     # FIRST, then scene elements
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(12, -12, 6), chrono.ChVector3d(0, 0, 0))  # view the gear pair
vis.AddTypicalLights()

# ---- time stepping parameters ----
time_step = 1e-3                                                     # 1 ms step
sim_end = 10.0                                                       # 10 s simulation
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))        # untagged cadence constant


while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)                                # advance one step
        if sys.GetChTime() >= sim_end:
            break
