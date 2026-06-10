import os
import math as m
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# System — NSC for rigid bodies; pure jointed MBS, no collision needed
sys = chrono.ChSystemNSC()

# Contact material (required by easy factory bodies)
mat = chrono.ChContactMaterialNSC()

# Gear radii
radA = 2.0                                                        # sun/central gear radius [m]
radB = 4.0                                                        # planet gear radius [m]

# Fixed truss (ground body) — large flat box serving as backdrop
mbody_truss = chrono.ChBodyEasyBox(20, 10, 2, 1000, True, False, mat)
sys.Add(mbody_truss)
mbody_truss.SetFixed(True)                                        # immovable anchor
mbody_truss.SetPos(chrono.ChVector3d(0, 0, 3))                    # behind the gears

# Texture for gears
vis_mat = chrono.ChVisualMaterial()
vis_mat.SetKdTexture(chrono.GetChronoDataFile('textures/pinkwhite.png'))

# Rotating bar — carrier for planet gear, pivots about truss Z axis at origin
# Bar placed so COM is at midspan between the two gear centers (0 and interaxis12)
interaxis12 = radA + radB                                         # distance between gear A and B axes
mbody_train = chrono.ChBodyEasyBox(8, 1.5, 1.0, 1000, True, False, mat)
sys.Add(mbody_train)
mbody_train.SetPos(chrono.ChVector3d(3, 0, 0))                    # centered on bar span

# Bar revolutes about the truss at world origin along Z axis
link_revoluteTT = chrono.ChLinkLockRevolute()
link_revoluteTT.Initialize(mbody_truss, mbody_train,
                           chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
sys.AddLink(link_revoluteTT)

# First gear (sun/central) — driven by the motor, at world origin
mbody_gearA = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radA, 0.5, 1000, True, False, mat)
sys.Add(mbody_gearA)
mbody_gearA.SetPos(chrono.ChVector3d(0, 0, -1))                   # slightly in front of truss
mbody_gearA.SetRot(chrono.QuatFromAngleX(m.pi / 2))               # lay cylinder flat (axis now Z)
mbody_gearA.GetVisualShape(0).SetMaterial(0, vis_mat)

# Shaft visual for gear A
mshaft_shape = chrono.ChVisualShapeCylinder(radA * 0.4, 13)
mbody_gearA.AddVisualShape(mshaft_shape, chrono.ChFramed(chrono.ChVector3d(0, 3.5, 0),
                            chrono.QuatFromAngleX(chrono.CH_PI_2)))

# Motor — prescribes constant speed on gear A against the fixed truss
# ChLinkMotorRotationSpeed is a FULL motor-link; no extra revolute needed
link_motor = chrono.ChLinkMotorRotationSpeed()
link_motor.Initialize(mbody_gearA, mbody_truss,
                      chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
link_motor.SetSpeedFunction(chrono.ChFunctionConst(6))            # 6 rad/s constant speed
sys.AddLink(link_motor)

# Second gear (planet) — orbits around gear A, carried on the bar
mbody_gearB = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radB, 0.4, 1000, True, False, mat)
sys.Add(mbody_gearB)
mbody_gearB.SetPos(chrono.ChVector3d(interaxis12, 0, -1))         # at interaxis distance
mbody_gearB.SetRot(chrono.QuatFromAngleX(m.pi / 2))               # lay cylinder flat
mbody_gearB.GetVisualShape(0).SetMaterial(0, vis_mat)

# Gear B revolutes about the bar at gear B's axis
link_revolute = chrono.ChLinkLockRevolute()
link_revolute.Initialize(mbody_gearB, mbody_train,
                         chrono.ChFramed(chrono.ChVector3d(interaxis12, 0, 0), chrono.QUNIT))
sys.AddLink(link_revolute)

# Gear constraint A-B: direct external mesh (no epicyclic)
# SetFrameShaft: Z-axis of the frame is the gear shaft axis;
# cylinders were built with Y-axis, rotated 90 deg, so shaft is now world Z
link_gearAB = chrono.ChLinkLockGear()
link_gearAB.Initialize(mbody_gearA, mbody_gearB, chrono.ChFramed())
link_gearAB.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))
link_gearAB.SetFrameShaft2(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))
link_gearAB.SetTransmissionRatio(radA / radB)                     # speed ratio A/B
link_gearAB.SetEnforcePhase(True)                                 # keep teeth phased
sys.AddLink(link_gearAB)

# Epicyclic ring gear: gear B also meshes with an inner (ring) gear on the truss
# The ring gear radius is radC = 2*radB + radA (geometry of epicyclic)
radC = 2 * radB + radA                                            # ring gear radius [m]
link_gearBC = chrono.ChLinkLockGear()
link_gearBC.Initialize(mbody_gearB, mbody_truss, chrono.ChFramed())
link_gearBC.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))
link_gearBC.SetFrameShaft2(chrono.ChFramed(chrono.ChVector3d(0, 0, -4), chrono.QUNIT))
link_gearBC.SetTransmissionRatio(radB / radC)                     # speed ratio B/C
link_gearBC.SetEpicyclic(True)                                    # inner (ring) gear teeth
sys.AddLink(link_gearBC)

# Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Epicyclic Gear System')
vis.Initialize()                                                  # FIRST, then add elements
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(12, 15, -20))
vis.AddTypicalLights()

# Use projected Euler integrator (matches demo)
sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_PROJECTED)

# Simulation parameters
time_step = 1e-3                                                  # physics timestep [s]
sim_end = 10.0                                                    # total simulation time [s]
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))      # cadence: steps per render frame


while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)
        if sys.GetChTime() >= sim_end:
            break
