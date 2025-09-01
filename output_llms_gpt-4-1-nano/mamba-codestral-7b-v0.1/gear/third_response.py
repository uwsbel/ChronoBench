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
radC = 2 * radB + radA
radD = 5
radE = 2

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

# Create a revolute joint between truss and rotating bar, allowing rotation along the Z-axis
link_revoluteTT = chrono.ChLinkLockRevolute()
link_revoluteTT.Initialize(mbody_truss, mbody_train, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
sys.AddLink(link_revoluteTT)

# Create the first gear
mbody_gearA = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radA, 0.5, 1000, True, False, mat)
sys.Add(mbody_gearA)
mbody_gearA.SetPos(chrono.ChVector3d(0, 0, -1))
mbody_gearA.SetRot(chrono.QuatFromAngleX(m.pi / 2))
mbody_gearA.GetVisualShape(0).SetMaterial(0, vis_mat)

# Adding a thin cylinder only for visualization purpose
mshaft_shape = chrono.ChVisualShapeCylinder(radA * 0.3, 10)
mbody_gearA.AddVisualShape(mshaft_shape, chrono.ChFramed(chrono.ChVector3d(0, 3.5, 0), chrono.ChQuaternion(chrono.CH_PI_2, 0, 0, 1)))

# Impose rotation speed on the first gear relative to the fixed truss
link_motor = chrono.ChLinkMotorRotationSpeed()
link_motor.Initialize(mbody_gearA, mbody_truss, chrono.ChFramed(chrono.ChVector3d(0,
print("error happened with only start ```python")