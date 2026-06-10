import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import math

# ---------------------------------------------------------------------
# Chrono system
# ---------------------------------------------------------------------
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

# ---------------------------------------------------------------------
# Parameters
# ---------------------------------------------------------------------
rod_length_1 = 2.0
rod_length_2 = 2.0
rod_radius = 0.08
rod_mass = 1.0

pivot_pos = chrono.ChVector3d(0, 0, 1)

# Initial absolute angles of the rods in the X-Y plane.
# Revolute axes are along global Z, so gravity in -Y causes planar motion.
theta1 = 0.0
theta2 = math.radians(-35.0)

# Correct inertia for a solid cylinder/rod whose local X axis is along its length.
Ixx = 0.5 * rod_mass * rod_radius * rod_radius
Iyy = (1.0 / 12.0) * rod_mass * (3.0 * rod_radius * rod_radius + rod_length_1 * rod_length_1)
Izz = Iyy
rod_inertia = chrono.ChVector3d(Ixx, Iyy, Izz)

# Rotation used to display Chrono's default cylinder along the body local X axis.
# This fixes the visualization so each cylinder represents a pendulum rod.
rod_visual_rot = chrono.QuatFromAngleY(math.pi / 2.0)

# ---------------------------------------------------------------------
# Ground body
# ---------------------------------------------------------------------
ground = chrono.ChBody()
ground.SetFixed(True)
ground.EnableCollision(False)
sys.AddBody(ground)

# Small visual marker at the fixed pivot
ground_marker = chrono.ChVisualShapeCylinder(0.15, 0.35)
ground_marker.SetColor(chrono.ChColor(0.2, 0.2, 0.2))
ground.AddVisualShape(
    ground_marker,
    chrono.ChFramed(pivot_pos, chrono.ChQuaterniond(1, 0, 0, 0))
)

# ---------------------------------------------------------------------
# Helper function for pendulum body creation
# ---------------------------------------------------------------------
def create_pendulum(name, com_pos, angle, color):
    body = chrono.ChBody()
    body.SetName(name)
    body.SetFixed(False)
    body.EnableCollision(False)

    body.SetMass(rod_mass)
    body.SetInertiaXX(rod_inertia)

    body.SetPos(com_pos)
    body.SetRot(chrono.QuatFromAngleZ(angle))

    rod_shape = chrono.ChVisualShapeCylinder(rod_radius, rod_length_1)
    rod_shape.SetColor(color)

    body.AddVisualShape(
        rod_shape,
        chrono.ChFramed(chrono.ChVector3d(0, 0, 0), rod_visual_rot)
    )

    sys.AddBody(body)
    return body

# ---------------------------------------------------------------------
# First pendulum
# ---------------------------------------------------------------------
dir1_x = math.cos(theta1)
dir1_y = math.sin(theta1)

pend1_com = chrono.ChVector3d(
    pivot_pos.x + 0.5 * rod_length_1 * dir1_x,
    pivot_pos.y + 0.5 * rod_length_1 * dir1_y,
    pivot_pos.z
)

joint12_pos = chrono.ChVector3d(
    pivot_pos.x + rod_length_1 * dir1_x,
    pivot_pos.y + rod_length_1 * dir1_y,
    pivot_pos.z
)

pend_1 = create_pendulum(
    "pendulum_1",
    pend1_com,
    theta1,
    chrono.ChColor(0.6, 0.0, 0.0)
)

# Revolute joint between ground and first pendulum
rev_1 = chrono.ChLinkLockRevolute()
rev_1.Initialize(
    ground,
    pend_1,
    chrono.ChFramed(pivot_pos, chrono.ChQuaterniond(1, 0, 0, 0))
)
sys.AddLink(rev_1)

# ---------------------------------------------------------------------
# Second pendulum
# ---------------------------------------------------------------------
dir2_x = math.cos(theta2)
dir2_y = math.sin(theta2)

pend2_com = chrono.ChVector3d(
    joint12_pos.x + 0.5 * rod_length_2 * dir2_x,
    joint12_pos.y + 0.5 * rod_length_2 * dir2_y,
    joint12_pos.z
)

pend_2 = create_pendulum(
    "pendulum_2",
    pend2_com,
    theta2,
    chrono.ChColor(0.0, 0.1, 0.7)
)

# Revolute joint between first and second pendulum
rev_2 = chrono.ChLinkLockRevolute()
rev_2.Initialize(
    pend_1,
    pend_2,
    chrono.ChFramed(joint12_pos, chrono.ChQuaterniond(1, 0, 0, 0))
)
sys.AddLink(rev_2)

# ---------------------------------------------------------------------
# Irrlicht visualization
# ---------------------------------------------------------------------
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Double pendulum - PyChrono")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3, 6))
vis.AddTypicalLights()

# ---------------------------------------------------------------------
# Simulation loop
# ---------------------------------------------------------------------
time_step = 1e-3
log_info = True

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    sys.DoStepDynamics(time_step)

    if log_info and sys.GetChTime() > 1.0:
        p1 = pend_1.GetPos()
        v1 = pend_1.GetPosDt()

        p2 = pend_2.GetPos()
        v2 = pend_2.GetPosDt()

        print("t =", sys.GetChTime())
        print("Pendulum 1 position:", p1.x, p1.y, p1.z)
        print("Pendulum 1 velocity:", v1.x, v1.y, v1.z)
        print("Pendulum 2 position:", p2.x, p2.y, p2.z)
        print("Pendulum 2 velocity:", v2.x, v2.y, v2.z)

        log_info = False