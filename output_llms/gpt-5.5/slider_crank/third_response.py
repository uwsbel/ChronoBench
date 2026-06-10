import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import matplotlib.pyplot as plt
import numpy as np

# If needed, set the Chrono data path explicitly, for example:
# chrono.SetChronoDataPath('path/to/chrono/data/')

sys = chrono.ChSystemNSC()

# Shared mechanism data
crank_center = chrono.ChVector3d(-1, 0.5, 0)
crank_rad = 0.4
crank_thick = 0.1
rod_length = 1.5

# ---------------------------------------------------------------------
# Create rigid bodies: floor/truss, crank, rod, piston
# ---------------------------------------------------------------------

# Floor / truss
mfloor = chrono.ChBodyEasyBox(3, 1, 3, 1000)
mfloor.SetPos(chrono.ChVector3d(0, -0.5, 0))
mfloor.SetFixed(True)
sys.Add(mfloor)

# Flywheel crank
mcrank = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, crank_rad, crank_thick, 1000)
mcrank.SetPos(crank_center + chrono.ChVector3d(0, 0, -0.1))

# ChBodyEasyCylinder is initially built with its axis along Y.
# Rotate it so that its cylinder axis is along Z.
mcrank.SetRot(chrono.Q_ROTATE_Y_TO_Z)
sys.Add(mcrank)

# Connecting rod
mrod = chrono.ChBodyEasyBox(rod_length, 0.1, 0.1, 1000)
mrod.SetPos(crank_center + chrono.ChVector3d(crank_rad + rod_length / 2, 0, 0))
sys.Add(mrod)

# Piston
mpiston = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.2, 0.3, 1000)
mpiston.SetPos(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0))

# Rotate piston cylinder axis from Y to X.
mpiston.SetRot(chrono.Q_ROTATE_Y_TO_X)
sys.Add(mpiston)

# ---------------------------------------------------------------------
# Joints / constraints
# ---------------------------------------------------------------------

# Crank-floor joint: rotational speed motor about the Z axis
my_motor = chrono.ChLinkMotorRotationSpeed()
my_motor.Initialize(
    mcrank,
    mfloor,
    chrono.ChFramed(crank_center)
)

# Angular speed: pi rad/s = 180 deg/s
my_angularspeed = chrono.ChFunctionConst(chrono.CH_PI)
my_motor.SetMotorFunction(my_angularspeed)
sys.Add(my_motor)

# Crank-rod joint:
# Modified from revolute hinge to spherical ball-and-socket joint.
mjointA = chrono.ChLinkLockSpherical()
mjointA.Initialize(
    mrod,
    mcrank,
    chrono.ChFramed(crank_center + chrono.ChVector3d(crank_rad, 0, 0))
)
sys.Add(mjointA)

# Rod-piston joint:
# Modified from revolute hinge to spherical ball-and-socket joint.
mjointB = chrono.ChLinkLockSpherical()
mjointB.Initialize(
    mpiston,
    mrod,
    chrono.ChFramed(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0))
)
sys.Add(mjointB)

# Piston-floor joint:
# Modified from prismatic joint to plane-plane planar joint.
# With the default frame orientation, the joint plane is the global X-Y plane,
# so the piston is constrained to remain in the X-Y plane and can rotate about Z.
mjointC = chrono.ChLinkLockPlanePlane()
mjointC.Initialize(
    mpiston,
    mfloor,
    chrono.ChFramed(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0))
)
sys.Add(mjointC)

# ---------------------------------------------------------------------
# Visualization
# ---------------------------------------------------------------------

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Crank demo - spherical joints and planar piston constraint")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1, 1, 3), chrono.ChVector3d(0, 1, 0))
vis.AddTypicalLights()

# ---------------------------------------------------------------------
# Data storage for plotting
# ---------------------------------------------------------------------

array_time = []
array_angle = []
array_pos = []
array_speed = []

# ---------------------------------------------------------------------
# Simulation loop
# ---------------------------------------------------------------------

step_size = 1e-3
end_time = 2.0

while vis.Run():
    # Store instantaneous values
    array_time.append(sys.GetChTime())
    array_angle.append(my_motor.GetMotorAngle())
    array_pos.append(mpiston.GetPos().x)
    array_speed.append(mpiston.GetPosDt().x)

    # Visualization
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Dynamics step
    sys.DoStepDynamics(step_size)

    # Stop after 2 seconds
    if sys.GetChTime() > end_time:
        vis.GetDevice().closeDevice()

# ---------------------------------------------------------------------
# Plot results
# ---------------------------------------------------------------------

fig, (ax1, ax2) = plt.subplots(2, sharex=True)

ax1.plot(array_angle, array_pos)
ax1.set_ylabel("position [m]")
ax1.grid(True)

ax2.plot(array_angle, array_speed, "r--")
ax2.set_ylabel("speed [m/s]")
ax2.set_xlabel("angle [rad]")
ax2.grid(True)

# Show pi-based ticks on the x-axis
ax2.set_xticks(np.linspace(0, 2 * np.pi, 5))
ax2.set_xticklabels(["0", r"$\pi/2$", r"$\pi$", r"$3\pi/2$", r"$2\pi$"])

plt.tight_layout()
plt.show()