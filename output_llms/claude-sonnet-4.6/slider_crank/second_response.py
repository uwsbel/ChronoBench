import pychrono.core as chrono  # Import the core Chrono library
import pychrono.irrlicht as chronoirr  # Import the Irrlicht visualization library for Chrono
import matplotlib.pyplot as plt  # Import matplotlib for plotting
import numpy as np  # Import numpy for numerical operations

# Initialize the Chrono simulation system with non-smooth contact (NSC) method
sys = chrono.ChSystemNSC()

# Define common parameters for the simulation
crank_center = chrono.ChVector3d(-1, 0.5, 0)  # Center of the crankshaft
crank_rad = 0.4  # Radius of the crankshaft (in meters)
crank_thick = 0.1  # Thickness of the crankshaft (in meters)
rod_length = 1.5  # Length of the connecting rod (in meters)

# Create the floor (truss) body, which is a box
mfloor = chrono.ChBodyEasyBox(3, 1, 3, 1000)
mfloor.SetPos(chrono.ChVector3d(0, -0.5, 0))
mfloor.SetFixed(True)
sys.Add(mfloor)

# Create the crank body, which is a cylinder
mcrank = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, crank_rad, crank_thick, 1000)
mcrank.SetPos(crank_center + chrono.ChVector3d(0, 0, -0.1))
mcrank.SetRot(chrono.Q_ROTATE_Y_TO_Z)
sys.Add(mcrank)

# Create the connecting rod, which is a box
mrod = chrono.ChBodyEasyBox(rod_length, 0.1, 0.1, 1000)
mrod.SetPos(crank_center + chrono.ChVector3d(crank_rad + rod_length / 2, 0, 0))
sys.Add(mrod)

# Create the piston, which is a cylinder
mpiston = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.2, 0.3, 1000)
mpiston.SetPos(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0))
mpiston.SetRot(chrono.Q_ROTATE_Y_TO_X)
sys.Add(mpiston)

# Create a motor to spin the crankshaft
my_motor = chrono.ChLinkMotorRotationSpeed()
my_motor.Initialize(mcrank, mfloor, chrono.ChFramed(crank_center))
my_angularspeed = chrono.ChFunctionConst(chrono.CH_PI)  # π rad/s
my_motor.SetMotorFunction(my_angularspeed)
sys.Add(my_motor)

# Create a revolute joint to connect the crank to the rod
mjointA = chrono.ChLinkLockRevolute()
mjointA.Initialize(mrod, mcrank, chrono.ChFramed(crank_center + chrono.ChVector3d(crank_rad, 0, 0)))
sys.Add(mjointA)

# Create a revolute joint to connect the rod to the piston
mjointB = chrono.ChLinkLockRevolute()
mjointB.Initialize(mpiston, mrod, chrono.ChFramed(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0)))
sys.Add(mjointB)

# Create a prismatic joint to connect the piston to the floor
mjointC = chrono.ChLinkLockPrismatic()
mjointC.Initialize(
    mpiston, mfloor,
    chrono.ChFramed(
        crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0),
        chrono.Q_ROTATE_Z_TO_X
    )
)
sys.Add(mjointC)

# Set up the Irrlicht visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Crank demo')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1, 1, 3), chrono.ChVector3d(0, 1, 0))
vis.AddTypicalLights()

# ── 3. Array Initialization for Plotting ──────────────────────────────────────
array_time  = []   # simulation time [s]
array_angle = []   # crank angle     [rad]
array_pos   = []   # piston position [m]
array_speed = []   # piston speed    [m/s]

# ── Simulation Loop ───────────────────────────────────────────────────────────
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(1e-3)   # 1 ms time step

    # ── 4. Collect Data During Simulation ─────────────────────────────────────
    array_time.append(sys.GetChTime())
    array_angle.append(my_motor.GetMotorAngle())   # cumulative crank angle [rad]
    array_pos.append(mpiston.GetPos().x)            # piston x-position      [m]
    array_speed.append(mpiston.GetPosDt().x)        # piston x-velocity      [m/s]

    # ── 5. Stop after 20 seconds ──────────────────────────────────────────────
    if sys.GetChTime() >= 20.0:
        break

# ── 6. Matplotlib Plotting ────────────────────────────────────────────────────
array_angle = np.array(array_angle)
array_pos   = np.array(array_pos)
array_speed = np.array(array_speed)

# Build π-based tick positions (0, π/2, π, 3π/2, …) up to the maximum angle
max_angle = array_angle[-1]
pi_ticks  = np.arange(0, max_angle + np.pi / 2, np.pi / 2)

# Create human-readable tick labels  (0, π/2, π, 3π/2, 2π, …)
def make_pi_label(val):
    n = round(val / (np.pi / 2))          # multiple of π/2
    if n == 0:
        return "0"
    elif n % 2 == 0:                       # multiples of π
        k = n // 2
        return f"{k}π" if k != 1 else "π"
    else:                                  # odd multiples of π/2
        return f"{n}π/2"

pi_labels = [make_pi_label(t) for t in pi_ticks]

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7))

# First subplot – position vs crank angle
ax1.plot(array_angle, array_pos, color='steelblue', linewidth=1.0)
ax1.set_xlabel('Crank angle [rad]')
ax1.set_ylabel('Position [m]')
ax1.set_title('Piston position vs Crank angle')
ax1.set_xticks(pi_ticks)
ax1.set_xticklabels(pi_labels)
ax1.grid(True)

# Second subplot – speed vs crank angle
ax2.plot(array_angle, array_speed, color='darkorange', linewidth=1.0)
ax2.set_xlabel('Crank angle [rad]')
ax2.set_ylabel('Speed [m/s]')
ax2.set_title('Piston speed vs Crank angle')
ax2.set_xticks(pi_ticks)
ax2.set_xticklabels(pi_labels)
ax2.grid(True)

plt.tight_layout()
plt.show()