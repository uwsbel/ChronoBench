import os
import math as m
import pychrono as chrono
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemNSC()                                           # NSC system for the gear train
chrono.SetChronoDataPath("/home/hongyu/Documents/chrono-900/data/")  # for textures/skybox
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -10, 0))       # Y-up world, g = 10

# contact material shared by all gear bodies (gear demo uses a default NSC material)
mat = chrono.ChContactMaterialNSC()

radA = 1.5      # sun gear radius (input2: was 2)
radB = 3.5      # planet gear radius (input2: was 4)
radC = 2 * radB + radA   # internal ring radius for the epicyclic set (concentric with the sun)

# --- truss: fixed support box (input2: 15 x 8 x 2, was 20 x 10 x 2) ---
mbody_truss = chrono.ChBodyEasyBox(15, 8, 2, 1000, True, False, mat) # visible truss box
sys.Add(mbody_truss)
mbody_truss.SetFixed(True)                                           # truss is the ground
mbody_truss.SetPos(chrono.ChVector3d(0, 0, 3))                       # behind the gear plane

# pinkwhite texture shared across the gear discs (canonical demo dressing)
vis_mat = chrono.ChVisualMaterial()
vis_mat.SetKdTexture(chrono.GetChronoDataFile("textures/pinkwhite.png"))

# --- sun gear A: driven by the motor, spins about world Z ---
mbody_gearA = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radA, 0.5, 1000, True, False, mat)
sys.Add(mbody_gearA)
mbody_gearA.SetPos(chrono.ChVector3d(0, 0, -1))                      # in the gear plane
mbody_gearA.SetRot(chrono.QuatFromAngleX(m.pi / 2))                  # lay disc so axis -> world Z
mbody_gearA.GetVisualShape(0).SetMaterial(0, vis_mat)                # pinkwhite texture

# decorative shaft cylinder on the sun so the rotation is visible (input2: radA*0.3, len 10)
mshaft_shape = chrono.ChVisualShapeCylinder(radA * 0.3, 10)
mbody_gearA.AddVisualShape(mshaft_shape, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(m.pi / 2)))

# --- planet gear B: rides on the carrier, meshes sun (external) and ring (internal) ---
interaxis12 = radA + radB                                            # center distance sun<->planet
mbody_gearB = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radB, 0.4, 1000, True, False, mat)
sys.Add(mbody_gearB)
mbody_gearB.SetPos(chrono.ChVector3d(interaxis12, 0, -2))            # input2: z = -2 (was -1)
mbody_gearB.SetRot(chrono.QuatFromAngleX(m.pi / 2))                  # lay disc so axis -> world Z
mbody_gearB.GetVisualShape(0).SetMaterial(0, vis_mat)                # pinkwhite texture

# --- carrier: arm holding the planet axle, rotates about the sun axis ---
mbody_train = chrono.ChBodyEasyBox(8, 1.5, 1.0, 1000, True, False, mat)   # visible carrier bar
sys.Add(mbody_train)
mbody_train.SetPos(chrono.ChVector3d(interaxis12 / 2, 0, -1.1))      # spans sun -> planet axle

# carrier revolute to the truss (carrier spins about the sun axis Z)
link_revoluteTT = chrono.ChLinkLockRevolute()
link_revoluteTT.Initialize(mbody_train, mbody_truss, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
sys.AddLink(link_revoluteTT)

# planet revolute attaches the planet to the CARRIER (so the axle orbits with the carrier)
link_revoluteBT = chrono.ChLinkLockRevolute()
link_revoluteBT.Initialize(mbody_gearB, mbody_train, chrono.ChFramed(chrono.ChVector3d(interaxis12, 0, 0), chrono.QUNIT))
sys.AddLink(link_revoluteBT)

# --- motor: drives the sun gear A against the truss (input2: speed 3, was 6) ---
link_motor = chrono.ChLinkMotorRotationSpeed()
link_motor.Initialize(mbody_gearA, mbody_truss, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
link_motor.SetSpeedFunction(chrono.ChFunctionConst(3))               # rad/s
sys.AddLink(link_motor)

# --- external mesh: sun A <-> planet B ---
link_gearAB = chrono.ChLinkLockGear()
link_gearAB.Initialize(mbody_gearA, mbody_gearB, chrono.ChFramed())
link_gearAB.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))
link_gearAB.SetFrameShaft2(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))
link_gearAB.SetTransmissionRatio(radA / radB)                        # external sun/planet ratio
link_gearAB.SetEnforcePhase(True)                                    # keep teeth phased
sys.AddLink(link_gearAB)

# --- internal mesh: planet B <-> ring (the fixed truss acts as the internal ring) ---
link_gearBC = chrono.ChLinkLockGear()
link_gearBC.Initialize(mbody_gearB, mbody_truss, chrono.ChFramed())
link_gearBC.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))
link_gearBC.SetFrameShaft2(chrono.ChFramed(chrono.ChVector3d(0, 0, -4), chrono.QUNIT))  # ring shaft behind gear plane
link_gearBC.SetTransmissionRatio(radB / radC)                        # internal planet/ring ratio
link_gearBC.SetEpicyclic(True)                                       # INTERNAL (ring) mesh
sys.AddLink(link_gearBC)

# projected integrator for stable gear meshing (every gear truth sets this)
sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_PROJECTED)

# --- Irrlicht visualization (Initialize first, then scene elements, NO grid) ---
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Epicyclic gears")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(12, 15, -20))                        # look onto the gear plane
vis.AddTypicalLights()

time_step = 1e-3                                                     # integration step
render_fps = 50.0                                                    # review video cadence
render_every = max(1, round(1.0 / (render_fps * time_step)))         # physics steps per rendered frame

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)
