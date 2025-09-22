import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math as m

# -----------------------------------------------------------------------------
# 1) CREATE THE PHYSICAL SYSTEM
# -----------------------------------------------------------------------------
sys = chrono.ChSystemNSC()

# Correct material class name for NSC contacts
mat = chrono.ChMaterialSurfaceNSC()

# -----------------------------------------------------------------------------
# 2) DEFINE THE NEW GEAR RADII
# -----------------------------------------------------------------------------
radA = 1.5        # was 2.0
radB = 3.5        # was 4.0

# -----------------------------------------------------------------------------
# 3) CREATE THE TRUSS (fixed)
#    Modified dimensions: 15 x 8 x 2  (was 20 x 10 x 2)
# -----------------------------------------------------------------------------
mbody_truss = chrono.ChBodyEasyBox(
    15, 8, 2,      # width, height, depth
    1000,          # density
    True, False,   # visualization on, collision off
    mat)
sys.Add(mbody_truss)
mbody_truss.SetFixed(True)
mbody_truss.SetPos(chrono.ChVector3d(0, 0, 3))

# A shared visual‐only material
vis_mat = chrono.ChVisualMaterial()
vis_mat.SetKdTexture(chrono.GetChronoDataFile('textures/pinkwhite.png'))

# -----------------------------------------------------------------------------
# 4) CREATE THE ROTATING “TRAIN” BAR
# -----------------------------------------------------------------------------
mbody_train = chrono.ChBodyEasyBox(
    8, 1.5, 1.0,   # width, height, depth
    1000,
    True, False,
    mat)
sys.Add(mbody_train)
mbody_train.SetPos(chrono.ChVector3d(3, 0, 0))

# Revolute joint between truss and bar
link_revoluteTT = chrono.ChLinkLockRevolute()
link_revoluteTT.Initialize(
    mbody_truss, mbody_train,
    chrono.ChCoordsysD(chrono.ChVector3d(0,0,0), chrono.QUNIT))
sys.AddLink(link_revoluteTT)

# -----------------------------------------------------------------------------
# 5) CREATE GEAR A (driven by motor)
# -----------------------------------------------------------------------------
mbody_gearA = chrono.ChBodyEasyCylinder(
    chrono.ChAxis_Y,  # cylinder axis
    radA,             # radius
    0.5,              # thickness
    1000, True, False,
    mat)
sys.Add(mbody_gearA)
mbody_gearA.SetPos(chrono.ChVector3d(0, 0, -1))
mbody_gearA.SetRot(chrono.QuatFromAngleX(m.pi/2))
mbody_gearA.GetVisualShape(0).SetMaterial(0, vis_mat)

# Thin cylinder for visual “shaft” on gear A
# Modified: radius = radA*0.3, length = 10  (was radA*0.4, length=13)
mshaft_shape = chrono.ChVisualShapeCylinder(radA*0.3, 10)
mbody_gearA.AddVisualShape(
    mshaft_shape,
    chrono.ChCoordsysD(
        chrono.ChVector3d(0, 3.5, 0),
        chrono.QuatFromAngleX(chrono.CH_C_PI_2)
    )
)

# Motor to spin gear A at constant speed
link_motor = chrono.ChLinkMotorRotationSpeed()
link_motor.Initialize(
    mbody_gearA, mbody_truss,
    chrono.ChCoordsysD(chrono.ChVector3d(0,0,0), chrono.QUNIT))
# Modified speed: 3 rad/s (was 6)
link_motor.SetSpeedFunction(chrono.ChFunctionConst(3))
sys.AddLink(link_motor)

# -----------------------------------------------------------------------------
# 6) CREATE GEAR B
# -----------------------------------------------------------------------------
interaxis12 = radA + radB
mbody_gearB = chrono.ChBodyEasyCylinder(
    chrono.ChAxis_Y,
    radB, 0.4,
    1000, True, False,
    mat)
sys.Add(mbody_gearB)
# Modified Z‐position = –2 (was –1)
mbody_gearB.SetPos(chrono.ChVector3d(interaxis12, 0, -2))
mbody_gearB.SetRot(chrono.QuatFromAngleX(m.pi/2))
mbody_gearB.GetVisualShape(0).SetMaterial(0, vis_mat)

# Revolute joint fixing gear B to the train bar
link_revolute = chrono.ChLinkLockRevolute()
link_revolute.Initialize(
    mbody_gearB, mbody_train,
    chrono.ChCoordsysD(chrono.ChVector3d(interaxis12, 0, 0), chrono.QUNIT))
sys.AddLink(link_revolute)

# -----------------------------------------------------------------------------
# 7) GEAR CONSTRAINT BETWEEN A AND B
# -----------------------------------------------------------------------------
link_gearAB = chrono.ChLinkLockGear()
link_gearAB.Initialize(
    mbody_gearA, mbody_gearB,
    chrono.ChCoordsysD())
link_gearAB.SetFrameShaft1(
    chrono.ChCoordsysD(chrono.ChVector3d(0,0,0),
                       chrono.QuatFromAngleX(-m.pi/2)))
link_gearAB.SetFrameShaft2(
    chrono.ChCoordsysD(chrono.ChVector3d(0,0,0),
                       chrono.QuatFromAngleX(-m.pi/2)))
link_gearAB.SetTransmissionRatio(radA / radB)
link_gearAB.SetEnforcePhase(True)
sys.AddLink(link_gearAB)

# -----------------------------------------------------------------------------
# 8) GEAR CONSTRAINT BETWEEN B AND TRUSS (epicyclic, internal teeth)
# -----------------------------------------------------------------------------
radC = 2*radB + radA
link_gearBC = chrono.ChLinkLockGear()
link_gearBC.Initialize(
    mbody_gearB, mbody_truss,
    chrono.ChCoordsysD())
link_gearBC.SetFrameShaft1(
    chrono.ChCoordsysD(chrono.ChVector3d(0,0,0),
                       chrono.QuatFromAngleX(-m.pi/2)))
# wheel C is fixed to truss at z = –4
link_gearBC.SetFrameShaft2(
    chrono.ChCoordsysD(chrono.ChVector3d(0,0,-4), chrono.QUNIT))
link_gearBC.SetTransmissionRatio(radB / radC)
link_gearBC.SetEpicyclic(True)
sys.AddLink(link_gearBC)

# -----------------------------------------------------------------------------
# 9) IRRLICHT VISUALIZATION SETUP
# -----------------------------------------------------------------------------
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024,768)
vis.SetWindowTitle('Gears and pulleys (modified sizes & speeds)')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(12,15,-20))
vis.AddTypicalLights()

# -----------------------------------------------------------------------------
# 10) CHOOSE AN INTEGRATOR AND RUN
# -----------------------------------------------------------------------------
sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_PROJECTED)

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(1e-3)