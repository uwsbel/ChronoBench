import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math as m

# -----------------------------------------------------------------------------
# Create Chrono physical system
# -----------------------------------------------------------------------------
sys = chrono.ChSystemNSC()

# Contact material shared by all collision-enabled bodies.
# Collision is disabled for most bodies in this kinematic gear example, but the
# material is still passed to the Easy* constructors.
mat = chrono.ChContactMaterialNSC()

# -----------------------------------------------------------------------------
# Basic parameters
# -----------------------------------------------------------------------------
radA = 1.5
radB = 3.5
radD = 5.0       # New bevel gear radius
radE = 2.0       # New pulley radius

# -----------------------------------------------------------------------------
# Visualization materials
# -----------------------------------------------------------------------------
vis_mat = chrono.ChVisualMaterial()
vis_mat.SetKdTexture(chrono.GetChronoDataFile("textures/pinkwhite.png"))

gearD_mat = chrono.ChVisualMaterial()
gearD_mat.SetDiffuseColor(chrono.ChColor(0.75, 0.20, 0.15))

pulleyE_mat = chrono.ChVisualMaterial()
pulleyE_mat.SetDiffuseColor(chrono.ChColor(0.15, 0.35, 0.85))

belt_mat = chrono.ChVisualMaterial()
belt_mat.SetDiffuseColor(chrono.ChColor(0.02, 0.02, 0.02))

# -----------------------------------------------------------------------------
# Fixed truss
# -----------------------------------------------------------------------------
mbody_truss = chrono.ChBodyEasyBox(
    15, 8, 2,
    1000,
    True,
    False,
    mat
)
sys.Add(mbody_truss)
mbody_truss.SetFixed(True)
mbody_truss.SetPos(chrono.ChVector3d(0, 0, 3))

# -----------------------------------------------------------------------------
# Rotating train / carrier
# -----------------------------------------------------------------------------
mbody_train = chrono.ChBodyEasyBox(
    8, 1.5, 1.0,
    1000,
    True,
    False,
    mat
)
sys.Add(mbody_train)
mbody_train.SetPos(chrono.ChVector3d(3, 0, 0))

# Revolute joint between truss and train, with vertical Z shaft
link_revoluteTT = chrono.ChLinkLockRevolute()
link_revoluteTT.Initialize(
    mbody_truss,
    mbody_train,
    chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT)
)
sys.AddLink(link_revoluteTT)

# -----------------------------------------------------------------------------
# Gear A
# -----------------------------------------------------------------------------
mbody_gearA = chrono.ChBodyEasyCylinder(
    chrono.ChAxis_Y,
    radA,
    0.5,
    1000,
    True,
    False,
    mat
)
sys.Add(mbody_gearA)
mbody_gearA.SetPos(chrono.ChVector3d(0, 0, -1))
mbody_gearA.SetRot(chrono.QuatFromAngleX(m.pi / 2))
mbody_gearA.GetVisualShape(0).SetMaterial(0, vis_mat)

# Thin shaft visualization for gear A
mshaft_shape = chrono.ChVisualShapeCylinder(radA * 0.3, 10)
mbody_gearA.AddVisualShape(
    mshaft_shape,
    chrono.ChFramed(
        chrono.ChVector3d(0, 3.5, 0),
        chrono.QuatFromAngleX(chrono.CH_PI_2)
    )
)

# Motor imposing angular speed on gear A relative to the fixed truss
link_motor = chrono.ChLinkMotorRotationSpeed()
link_motor.Initialize(
    mbody_gearA,
    mbody_truss,
    chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT)
)
link_motor.SetSpeedFunction(chrono.ChFunctionConst(3.0))
sys.AddLink(link_motor)

# -----------------------------------------------------------------------------
# Gear B
# -----------------------------------------------------------------------------
interaxis12 = radA + radB

mbody_gearB = chrono.ChBodyEasyCylinder(
    chrono.ChAxis_Y,
    radB,
    0.4,
    1000,
    True,
    False,
    mat
)
sys.Add(mbody_gearB)
mbody_gearB.SetPos(chrono.ChVector3d(interaxis12, 0, -2))
mbody_gearB.SetRot(chrono.QuatFromAngleX(m.pi / 2))
mbody_gearB.GetVisualShape(0).SetMaterial(0, vis_mat)

# Revolute joint between gear B and the rotating train
link_revolute = chrono.ChLinkLockRevolute()
link_revolute.Initialize(
    mbody_gearB,
    mbody_train,
    chrono.ChFramed(chrono.ChVector3d(interaxis12, 0, 0), chrono.QUNIT)
)
sys.AddLink(link_revolute)

# Gear constraint between gear A and gear B
link_gearAB = chrono.ChLinkLockGear()
link_gearAB.Initialize(mbody_gearA, mbody_gearB, chrono.ChFramed())
link_gearAB.SetFrameShaft1(
    chrono.ChFramed(
        chrono.ChVector3d(0, 0, 0),
        chrono.QuatFromAngleX(-m.pi / 2)
    )
)
link_gearAB.SetFrameShaft2(
    chrono.ChFramed(
        chrono.ChVector3d(0, 0, 0),
        chrono.QuatFromAngleX(-m.pi / 2)
    )
)
link_gearAB.SetTransmissionRatio(radA / radB)
link_gearAB.SetEnforcePhase(True)
sys.AddLink(link_gearAB)

# Gear constraint between gear B and the fixed internal ring gear C.
# Here the truss acts as the large fixed wheel C.
radC = 2 * radB + radA

link_gearBC = chrono.ChLinkLockGear()
link_gearBC.Initialize(mbody_gearB, mbody_truss, chrono.ChFramed())
link_gearBC.SetFrameShaft1(
    chrono.ChFramed(
        chrono.ChVector3d(0, 0, 0),
        chrono.QuatFromAngleX(-m.pi / 2)
    )
)
link_gearBC.SetFrameShaft2(
    chrono.ChFramed(
        chrono.ChVector3d(0, 0, -4),
        chrono.QUNIT
    )
)
link_gearBC.SetTransmissionRatio(radB / radC)
link_gearBC.SetEpicyclic(True)
sys.AddLink(link_gearBC)

# =============================================================================
# New mechanism: bevel gear D and pulley E
# =============================================================================

# -----------------------------------------------------------------------------
# Gear D: bevel gear represented by a cylinder for simplified visualization.
# It is placed at (-10, 0, -9), rotated 90 deg around Z.
# Since the cylinder is created with local Y as its axis, rotating the body
# by +90 deg around Z makes its shaft horizontal, approximately along global X.
# -----------------------------------------------------------------------------
mbody_gearD = chrono.ChBodyEasyCylinder(
    chrono.ChAxis_Y,
    radD,
    0.6,
    1000,
    True,
    False,
    mat
)
sys.Add(mbody_gearD)
mbody_gearD.SetPos(chrono.ChVector3d(-10, 0, -9))
mbody_gearD.SetRot(chrono.QuatFromAngleZ(m.pi / 2))
mbody_gearD.GetVisualShape(0).SetMaterial(0, gearD_mat)

# Revolute joint between gear D and the truss.
# ChLinkLockRevolute uses the Z axis of the joint frame as the revolute axis.
# Rotate the joint frame by +90 deg around Y so local Z maps to global X.
shaft_horizontal_X = chrono.QuatFromAngleY(m.pi / 2)

link_revoluteD = chrono.ChLinkLockRevolute()
link_revoluteD.Initialize(
    mbody_gearD,
    mbody_truss,
    chrono.ChFramed(chrono.ChVector3d(-10, 0, -9), shaft_horizontal_X)
)
sys.AddLink(link_revoluteD)

# -----------------------------------------------------------------------------
# 1:1 bevel gear constraint between gear A and gear D.
# Gear A shaft is vertical. Gear D shaft is horizontal.
# The shaft frame of D is expressed in gear D local coordinates.
# -----------------------------------------------------------------------------
q_gearA_shaft_local = chrono.QuatFromAngleX(-m.pi / 2)

# Gear D body is rotated by Qz(+90).  The desired absolute shaft frame has its
# local Z axis along global X, i.e. Qy(+90).  Express this in gear D coordinates.
q_gearD_shaft_local = chrono.QuatFromAngleZ(-m.pi / 2) * chrono.QuatFromAngleY(m.pi / 2)

link_gearAD = chrono.ChLinkLockGear()
link_gearAD.Initialize(mbody_gearA, mbody_gearD, chrono.ChFramed())
link_gearAD.SetFrameShaft1(
    chrono.ChFramed(
        chrono.ChVector3d(0, 0, 0),
        q_gearA_shaft_local
    )
)
link_gearAD.SetFrameShaft2(
    chrono.ChFramed(
        chrono.ChVector3d(0, 0, 0),
        q_gearD_shaft_local
    )
)
link_gearAD.SetTransmissionRatio(1.0)
link_gearAD.SetEnforcePhase(True)
sys.AddLink(link_gearAD)

# -----------------------------------------------------------------------------
# Pulley E
# -----------------------------------------------------------------------------
mbody_pulleyE = chrono.ChBodyEasyCylinder(
    chrono.ChAxis_Y,
    radE,
    0.6,
    1000,
    True,
    False,
    mat
)
sys.Add(mbody_pulleyE)
mbody_pulleyE.SetPos(chrono.ChVector3d(-10, -11, -9))
mbody_pulleyE.SetRot(chrono.QuatFromAngleZ(m.pi / 2))
mbody_pulleyE.GetVisualShape(0).SetMaterial(0, pulleyE_mat)

# Revolute joint between pulley E and truss, also along the horizontal X axis
link_revoluteE = chrono.ChLinkLockRevolute()
link_revoluteE.Initialize(
    mbody_pulleyE,
    mbody_truss,
    chrono.ChFramed(chrono.ChVector3d(-10, -11, -9), shaft_horizontal_X)
)
sys.AddLink(link_revoluteE)

# -----------------------------------------------------------------------------
# Synchronous belt constraint between gear D and pulley E.
# For a no-slip timing belt, the angular speed ratio is related to pulley radii.
# -----------------------------------------------------------------------------
link_pulleyDE = chrono.ChLinkLockPulley()
link_pulleyDE.Initialize(mbody_gearD, mbody_pulleyE, chrono.ChFramed())
link_pulleyDE.SetFrameShaft1(
    chrono.ChFramed(
        chrono.ChVector3d(0, 0, 0),
        q_gearD_shaft_local
    )
)
link_pulleyDE.SetFrameShaft2(
    chrono.ChFramed(
        chrono.ChVector3d(0, 0, 0),
        q_gearD_shaft_local
    )
)
link_pulleyDE.SetTransmissionRatio(radD / radE)
link_pulleyDE.SetEnforcePhase(True)
sys.AddLink(link_pulleyDE)

# -----------------------------------------------------------------------------
# Simplified belt visualization
# -----------------------------------------------------------------------------
# The belt is drawn as two fixed black slender boxes approximating the two
# external tangent spans, plus thin black disks behind the wheels to suggest
# wrap-around.  These are visual-only bodies with collision disabled.

def add_belt_span(p1, p2, thickness=0.14):
    """Add a fixed visual belt span between two 3D points."""
    dx = p2.x - p1.x
    dy = p2.y - p1.y
    dz = p2.z - p1.z

    length = m.sqrt(dx * dx + dy * dy + dz * dz)

    mid = chrono.ChVector3d(
        0.5 * (p1.x + p2.x),
        0.5 * (p1.y + p2.y),
        0.5 * (p1.z + p2.z)
    )

    # Span lies in the Y-Z plane.  The EasyBox is long along local Y.
    angle_x = m.atan2(dz, dy)

    belt_body = chrono.ChBodyEasyBox(
        thickness,
        length,
        thickness,
        1000,
        True,
        False,
        mat
    )
    sys.Add(belt_body)
    belt_body.SetFixed(True)
    belt_body.SetPos(mid)
    belt_body.SetRot(chrono.QuatFromAngleX(angle_x))
    belt_body.GetVisualShape(0).SetMaterial(0, belt_mat)
    return belt_body

# Compute approximate external tangent points in the Y-Z plane
centerD_y = 0.0
centerD_z = -9.0
centerE_y = -11.0
centerE_z = -9.0
center_distance = abs(centerE_y - centerD_y)

# External tangent normal components for unequal radii
ny = -(radD - radE) / center_distance
nz = m.sqrt(max(0.0, 1.0 - ny * ny))

# Two belt sides
for sign in [1.0, -1.0]:
    n_y = ny
    n_z = sign * nz

    # Tangent points using a simple external tangent construction
    pD = chrono.ChVector3d(
        -10,
        centerD_y - radD * n_y,
        centerD_z - radD * n_z
    )
    pE = chrono.ChVector3d(
        -10,
        centerE_y - radE * n_y,
        centerE_z - radE * n_z
    )
    add_belt_span(pD, pE)

# Add thin black disks just behind gear D and pulley E as wrap visualization
belt_wrapD = chrono.ChBodyEasyCylinder(
    chrono.ChAxis_X,
    radD + 0.08,
    0.05,
    1000,
    True,
    False,
    mat
)
sys.Add(belt_wrapD)
belt_wrapD.SetFixed(True)
belt_wrapD.SetPos(chrono.ChVector3d(-10.35, 0, -9))
belt_wrapD.GetVisualShape(0).SetMaterial(0, belt_mat)

belt_wrapE = chrono.ChBodyEasyCylinder(
    chrono.ChAxis_X,
    radE + 0.08,
    0.05,
    1000,
    True,
    False,
    mat
)
sys.Add(belt_wrapE)
belt_wrapE.SetFixed(True)
belt_wrapE.SetPos(chrono.ChVector3d(-10.35, -11, -9))
belt_wrapE.GetVisualShape(0).SetMaterial(0, belt_mat)

# -----------------------------------------------------------------------------
# Irrlicht visualization
# -----------------------------------------------------------------------------
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Gears, bevel gear, and timing belt pulley")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(12, 15, -25), chrono.ChVector3d(-5, -4, -5))
vis.AddTypicalLights()

# -----------------------------------------------------------------------------
# Integrator / timestepper
# -----------------------------------------------------------------------------
sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_PROJECTED)

# -----------------------------------------------------------------------------
# Simulation loop
# -----------------------------------------------------------------------------
time_step = 1e-3

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    sys.DoStepDynamics(time_step)