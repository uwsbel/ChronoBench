import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math as m

# Create a Chrono::Engine physical system
sys = chrono.ChSystemNSC()

# Create contact material shared among all bodies
mat = chrono.ChContactMaterialNSC()

# Create all rigid bodies with specific dimensions
radA = 1.5  # Defining radius for first gear
radB = 3.5  # Defining radius for second gear

# Create the truss
mbody_truss = chrono.ChBodyEasyBox(15, 8, 2,  # Modified dimensions
                1000,
                True,
                False,
                mat)
sys.Add(mbody_truss)
mbody_truss.SetFixed(True)
mbody_truss.SetPos(chrono.ChVector3d(0, 0, 3))

# Shared visualization material for enhanced aesthetics
vis_mat = chrono.ChVisualMaterial()
vis_mat.SetKdTexture(chrono.GetChronoDataFile('textures/pinkwhite.png'))

# Create the rotating bar support for the two epicycloidal wheels
mbody_train = chrono.ChBodyEasyBox(8, 1.5, 1.0,
                1000,
                True,
                False,
                mat)
sys.Add(mbody_train)
mbody_train.SetPos(chrono.ChVector3d(3, 0, 0))

# Create a revolute joint between truss and rotating bar
link_revoluteTT = chrono.ChLinkLockRevolute()
link_revoluteTT.Initialize(mbody_truss, mbody_train,
                           chrono.ChFramed(chrono.ChVector3d(0,0,0),
                chrono.QUNIT))
sys.AddLink(link_revoluteTT)

# Create the first gear
mbody_gearA = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y,
                radA, 0.5,
                1000, True, False, mat)
sys.Add(mbody_gearA)
mbody_gearA.SetPos(chrono.ChVector3d(0, 0, -1))
mbody_gearA.SetRot(chrono.QuatFromAngleX(m.pi / 2))
mbody_gearA.GetVisualShape(0).SetMaterial(0, vis_mat)

# Adding a thin cylinder only for visualization purpose
mshaft_shape = chrono.ChVisualShapeCylinder(radA * 0.3, 10)  # Modified size
mbody_gearA.AddVisualShape(mshaft_shape, chrono.ChFramed(chrono.ChVector3d(0, 3.5, 0),
                chrono.QuatFromAngleX(chrono.CH_PI_2)))

# Impose rotation speed on the first gear
link_motor = chrono.ChLinkMotorRotationSpeed()
link_motor.Initialize(mbody_gearA, mbody_truss,
                      chrono.ChFramed(chrono.ChVector3d(0, 0, 0),
                chrono.QUNIT))
link_motor.SetSpeedFunction(chrono.ChFunctionConst(3))  # Modified speed
sys.AddLink(link_motor)

# Create the second gear
interaxis12 = radA + radB
mbody_gearB = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y,
                radB, 0.4,
                1000, True, False, mat)
sys.Add(mbody_gearB)
mbody_gearB.SetPos(chrono.ChVector3d(interaxis12, 0, -2))  # Modified position
mbody_gearB.SetRot(chrono.QuatFromAngleX(m.pi / 2))
mbody_gearB.GetVisualShape(0).SetMaterial(0, vis_mat)

# Fix second gear to the rotating bar
link_revolute = chrono.ChLinkLockRevolute()
link_revolute.Initialize(mbody_gearB, mbody_train,
                         chrono.ChFramed(chrono.ChVector3d(interaxis12, 0, 0), chrono.QUNIT))
sys.AddLink(link_revolute)

# Create the gear constraint between the two gears
link_gearAB = chrono.ChLinkLockGear()
link_gearAB.Initialize(mbody_gearA, mbody_gearB, chrono.ChFramed())
link_gearAB.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))
link_gearAB.SetFrameShaft2(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))
link_gearAB.SetTransmissionRatio(radA / radB)
link_gearAB.SetEnforcePhase(True)
sys.AddLink(link_gearAB)

# Create the gear constraint between second gear B and a large wheel C
radC = 2 * radB + radA
link_gearBC = chrono.ChLinkLockGear()
link_gearBC.Initialize(mbody_gearB, mbody
print("error happened with only start ```python")