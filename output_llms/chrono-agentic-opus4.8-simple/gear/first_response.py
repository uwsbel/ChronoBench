import os
import math as m
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemNSC()                                            # NSC physical system

mat = chrono.ChContactMaterialNSC()                                   # shared contact material for the bodies

radA = 2                                                              # radius of the first (driving) gear
radB = 4                                                              # radius of the second (driven) gear

# ...the fixed truss
mbody_truss = chrono.ChBodyEasyBox(20, 10, 2, 1000, True, False, mat)  # truss box (visualized, no collision)
sys.Add(mbody_truss)                                                  # add truss to the system
mbody_truss.SetFixed(True)                                            # truss is the fixed reference
mbody_truss.SetPos(chrono.ChVector3d(0, 0, 3))                        # place the truss above the gears

# shared visualization material for the gears
vis_mat = chrono.ChVisualMaterial()                                  # texture material
vis_mat.SetKdTexture(chrono.GetChronoDataFile('textures/pinkwhite.png'))  # checker-like texture

# ...the rotating bar that supports the epicyclic wheels
mbody_train = chrono.ChBodyEasyBox(8, 1.5, 1.0, 1000, True, False, mat)  # rotating bar
sys.Add(mbody_train)                                                  # add the bar to the system
mbody_train.SetPos(chrono.ChVector3d(3, 0, 0))                        # bar spans between the two gear axes

# ...the bar rotates relative to the truss about the Z axis at the origin
link_revoluteTT = chrono.ChLinkLockRevolute()                        # revolute truss<->bar
link_revoluteTT.Initialize(mbody_truss, mbody_train,
                           chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))  # hinge about world +Z
sys.AddLink(link_revoluteTT)                                         # add the hinge

# ...the first gear (driving wheel A)
mbody_gearA = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radA, 0.5, 1000, True, False, mat)  # cylinder, Y axis
sys.Add(mbody_gearA)                                                 # add gear A
mbody_gearA.SetPos(chrono.ChVector3d(0, 0, -1))                      # gear A at the central axis
mbody_gearA.SetRot(chrono.QuatFromAngleX(m.pi / 2))                  # lay the cylinder so its axis is world Z
mbody_gearA.GetVisualShape(0).SetMaterial(0, vis_mat)               # apply the texture to gear A

# ...impose a constant rotation speed between gear A and the fixed truss (the gear motor)
link_motor = chrono.ChLinkMotorRotationSpeed()                      # speed motor = full motor-link
link_motor.Initialize(mbody_gearA, mbody_truss,
                       chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))  # spin axis = world +Z
link_motor.SetSpeedFunction(chrono.ChFunctionConst(6))             # constant 6 rad/s
sys.AddLink(link_motor)                                             # add the motor

# ...the second gear (driven wheel B), carried on the rotating bar
interaxis12 = radA + radB                                           # distance between the two gear axes
mbody_gearB = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radB, 0.4, 1000, True, False, mat)  # cylinder, Y axis
sys.Add(mbody_gearB)                                                # add gear B
mbody_gearB.SetPos(chrono.ChVector3d(interaxis12, 0, -1))          # gear B at the outer axis
mbody_gearB.SetRot(chrono.QuatFromAngleX(m.pi / 2))               # lay the cylinder so its axis is world Z
mbody_gearB.GetVisualShape(0).SetMaterial(0, vis_mat)             # apply the texture to gear B

# ...gear B spins on the rotating bar via a revolute joint
link_revolute = chrono.ChLinkLockRevolute()                       # revolute gearB<->bar
link_revolute.Initialize(mbody_gearB, mbody_train,
                         chrono.ChFramed(chrono.ChVector3d(interaxis12, 0, 0), chrono.QUNIT))  # axis world +Z
sys.AddLink(link_revolute)                                        # add the revolute

# ...the gear constraint between wheels A and B (transmission ratio = radA/radB)
link_gearAB = chrono.ChLinkLockGear()                             # gear mesh A<->B
link_gearAB.Initialize(mbody_gearA, mbody_gearB, chrono.ChFramed())  # initial frame position is irrelevant
link_gearAB.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))  # A shaft = local Z
link_gearAB.SetFrameShaft2(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))  # B shaft = local Z
link_gearAB.SetTransmissionRatio(radA / radB)                    # speed ratio from the two radii
link_gearAB.SetEnforcePhase(True)                               # keep the teeth phased
sys.AddLink(link_gearAB)                                        # add the gear constraint

# ...the epicyclic constraint between wheel B and the fixed internal-teeth wheel C (the truss)
radC = 2 * radB + radA                                          # pitch radius of the internal ring wheel
link_gearBC = chrono.ChLinkLockGear()                           # gear mesh B<->ring C (= truss)
link_gearBC.Initialize(mbody_gearB, mbody_truss, chrono.ChFramed())  # truss acts as the fixed ring
link_gearBC.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))  # B shaft = local Z
link_gearBC.SetFrameShaft2(chrono.ChFramed(chrono.ChVector3d(0, 0, -4), chrono.QUNIT))  # ring axis at center
link_gearBC.SetTransmissionRatio(radB / radC)                   # ratio against the ring radius
link_gearBC.SetEpicyclic(True)                                # internal-teeth (ring) gear
sys.AddLink(link_gearBC)                                       # add the epicyclic constraint

# Create the Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()                       # Irrlicht renderer
vis.AttachSystem(sys)                                          # bind the system's visual assets
vis.SetWindowSize(1024, 768)                                   # window size
vis.SetWindowTitle('Epicyclic gears')                          # window title
vis.Initialize()                                              # create the device FIRST
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  # logo overlay
vis.AddSkyBox()                                               # sky box
vis.AddCamera(chrono.ChVector3d(12, 15, -20))                # camera eye position
vis.AddTypicalLights()                                        # standard two-light setup

sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_PROJECTED)  # stable integrator for the gears

time_step = 1e-3                                              # physics step
sim_end = 10.0                                               # simulated duration (s)
render_fps = 50.0                                            # target frames per second
render_every = max(1, round(1.0 / (render_fps * time_step)))  # physics steps between rendered frames
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)
        if sys.GetChTime() >= sim_end:
            break
