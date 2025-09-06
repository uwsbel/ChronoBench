import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math as m

# Create a Chrono::Engine physical system
sys = chrono.ChSystemNSC()

# Create contact material shared among all bodies
mat = chrono.ChContactMaterialNSC()

# Create all rigid bodies with specific dimensions
radA = 1.5
radB = 3.5
radD = 5.0  # Radius for the bevel gear
radE = 2.0  # Radius for the pulley

# Create the truss
mbody_truss = chrono.ChBodyEasyBox(15, 8, 2, 1000, True, False, mat)
sys.Add(mbody_truss)
mbody_truss.SetFixed(True)
mbody_truss.SetPos(chrono.ChVector3d(0, 0, 3))

# Shared visualization material for enhanced aesthetics
vis_mat = chrono.ChVisualMaterial()
vis_mat.SetKdTexture(chrono.GetChronoDataFile('textures/pinkwhite.png'))

# Create the rotating bar support for the two epicycloidal wheels
mbody_train = chrono.ChBodyEasyBox(8, 1.5, 1.0, 1000, True, False, mat)
sys.Add(mbody_train)
mbody_train.SetPos(chrono.ChVector3d(3, 0, 0))

# Create the first gear
mbody_gearA = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radA, 0.5, 1000, True, False, mat)
mbody_gearA.SetPos(chrono.ChVector3d(0, 0, -1))
mbody_gearA.SetRot(chrono.QuatFromAngleX(m.pi / 2))
mbody_gearA.GetVisualShape(0).SetMaterial(0, vis_mat)
mshaft_shape = chrono.ChVisualShapeCylinder(radA * 0.3, 10)
mbody_gearA.AddVisualShape(mshaft_shape, chrono.ChFramed(chrono.ChVector3d(0, 3.5, 0), chrono.QuatFromAngleX(chrono.CH_PI_2))

# Create the bevel gear (gear D)
mbody_gearD = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radD, 0.5, 1000, True, False, mat)
mbody_gearD.SetPos(chrono.ChVector3d(-10, 0, -9))
mbody_gearD.SetRot(chrono.QuatFromAngleX(m.pi / 2))

# Create the revolute joint between truss and bevel gear D
link_revoluteTD = chrono.ChLinkLockRevolute()
link_revoluteTD.Initialize(mbody_truss, mbody_gearD, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
sys.AddLink(link_revoluteTD)

# Create the second gear
mbody_gearB = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radB, 0.4, 1000, True, False, mat)
mbody_gearB.SetPos(chrono.ChVector3d(chrono.ChVector3d(radA + radB, 0, -2))
mbody_gearB.SetRot(chrono.QuatFromAngleX(m.pi / 2))

# Create the pulley (pulley E)
mbody_pulleyE = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radE, 0.5, 1000, True, False, mat)
mbody_pulleyE.SetPos(chrono.ChVector3d(-10, -11, -9))
mbody_pulleyE.SetRot(chrono.QuatFromAngleX(m.pi / 2))

# Create the revolute joint between truss and pulley E
link_revoluteTE = chrono.ChLinkLockRevolute()
link_revoluteTE.Initialize(mbody_truss, mbody_pulleyE, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
sys.AddLink(link_revoluteTE)

# Create the gear constraint between gear A and bevel gear D
link_gearAD = chrono.ChLinkLockGear()
link_gearAD.Initialize(mbody_gearA, mbody_gearD, chrono.ChFramed())
link_gearAD.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))
link_gearAD.SetFrameShaft2(chrono.ChFramed(chrono.ChVector3d(0, 0, -4), chrono.QUNIT))
link_gearAD.SetTransmissionRatio(1.0)  # 1:1 gear ratio between gear A and D

# Create the synchro belt constraint between gear D and pulley E
link_synchroBelt = chrono.ChLinkSynchroBelt()
link_synchroBelt.Initialize(mbody_gearD, mbody_pulleyE, chrono.ChFramed(chrono.ChVector3d(0, 0, -4), chrono.ChFramed(chrono.ChVector3d(0, 0, -4), chrono.QUNIT))
sys.AddLink(link_synchroBelt)

# Create the gear constraint between bevel gear D and pulley E
link_gearDE = chrono.ChLinkLockGear()
link_gearDE.Initialize(mbody_gearD, mbody_pulleyE, chrono.ChFramed())
link_gearDE.SetFrameShaft1(chrono.ChFramed(chrono.ChVector3d(0, 0, -4), chrono.QuatFromAngleX(-m.pi / 2)))
link_gearDE.SetFrameShaft2(chrono.ChFramed(chrono.ChVector3d(0, 0, -4), chrono.QUNIT))
link_gearDE.SetTransmissionRatio(1.0)  # 1:1 gear ratio between D and E

# Create the Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Gears and pulleys')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(12, 15, -20))
vis.AddTypicalLights()

# Set integrator type
sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_PROJECTED)

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(1e-3)

# Visualization enhancements for the bevel gear (gear D) and pulley (pulley E)
# Create visual shapes for the bevel gear and pulley
bevel_gear_shape = chrono.ChVisualShapeCylinder(radD, 0.5, 10)
pulley_shape = chrono.ChVisualShapeCylinder(radE, 0.5, 10)

# Apply visual shapes to the bodies
mbody_gearD.AddVisualShape(bevel_gear_shape, chrono.ChFramed(chrono.ChVector3d(0, 0, -4), chrono.QuatFromAngleX(chrono.CH_PI_2)))
mbody_pulleyE.AddVisualShape(pulley_shape, chrono.ChFramed(chrono.ChVector3d(0, 0, -4), chrono.QuatFromAngleX(chrono.CH_PI_2)))

# Simulation loop (continued)
while vis.Run():
    vis.BeginScene()
    vis.Render()
    # Visualization enhancements for the belt can be added here if necessary
    vis.EndScene()
    sys.DoStepDynamics(1e-3)