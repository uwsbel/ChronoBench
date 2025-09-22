import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import matplotlib.pyplot as plt
import numpy as np

# Initialize the Chrono simulation system
sys = chrono.ChSystemNSC()

# Define common parameters
crank_center = chrono.ChVector3d(-1, 0.5, 0)
crank_rad = 0.4
crank_thick = 0.1
rod_length = 1.5

# Create floor (truss)
mfloor = chrono.ChBodyEasyBox(3, 1, 3, 1000)
mfloor.SetPos(chrono.ChVector3d(0, -0.5, 0))
mfloor.SetFixed(True)
sys.Add(mfloor)

# Create crank (corrected position - removed Z offset)
mcrank = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, crank_rad, crank_thick, 1000)
mcrank.SetPos(crank_center)  # Fixed: Removed unnecessary Z offset
mcrank.SetRot(chrono.Q_ROTATE_Y_TO_Z)
sys.Add(mcrank)

# Create connecting rod
mrod = chrono.ChBodyEasyBox(rod_length, 0.1, 0.1, 1000)
mrod.SetPos(crank_center + chrono.ChVector3d(crank_rad + rod_length / 2, 0, 0))
sys.Add(mrod)

# Create piston
mpiston = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.2, 0.3, 1000)
mpiston.SetPos(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0))
mpiston.SetRot(chrono.Q_ROTATE_Y_TO_X)
sys.Add(mpiston)

# Create motor to spin crankshaft (fixed frame initialization)
my_motor = chrono.ChLinkMotorRotationSpeed()
my_motor.Initialize(mcrank, mfloor, chrono.ChFrameD(crank_center))  # Fixed: ChFrameD instead of ChFramed
my_angularspeed = chrono.ChFunctionConst(chrono.CH_PI)
my_motor.SetMotorFunction(my_angularspeed)
sys.Add(my_motor)

# Create revolute joint: crank to rod (fixed frame)
mjointA = chrono.ChLinkLockRevolute()
mjointA.Initialize(mrod, mcrank, chrono.ChFrameD(crank_center + chrono.ChVector3d(crank_rad, 0, 0)))  # Fixed frame
sys.Add(mjointA)

# Create revolute joint: rod to piston (fixed frame)
mjointB = chrono.ChLinkLockRevolute()
mjointB.Initialize(mpiston, mrod, chrono.ChFrameD(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0)))  # Fixed frame
sys.Add(mjointB)

# Create prismatic joint: piston to floor (fixed orientation)
mjointC = chrono.ChLinkLockPrismatic()
mjointC.Initialize(
    mpiston, 
    mfloor, 
    chrono.ChFrameD(
        crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0),
        chrono.QUNIT  # Fixed: Identity rotation for X-axis motion
    )
)
sys.Add(mjointC)

# Initialize visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Crank Demo')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1, 1, 3), chrono.ChVector3d(0, 1, 0))
vis.AddTypicalLights()

# Initialize data collection arrays
array_time = []
array_angle = []
array_pos = []
array_speed = []

# Simulation parameters
simulation_duration = 20.0  # Stop after 20 seconds

# Run simulation
while vis.Run():
    current_time = sys.GetChTime()
    
    # Stop condition after 20 seconds
    if current_time >= simulation_duration:
        break
    
    # Collect data
    array_time.append(current_time)
    array_angle.append(my_motor.GetMotorRot())  # Crank angle
    array_pos.append(mpiston.GetPos().x)        # Piston X position
    array_speed.append(mpiston.GetPosDt().x)     # Piston X velocity
    
    # Visualization and stepping
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(1e-3)

# Convert to numpy arrays for processing
array_angle = np.array(array_angle)
array_pos = np.array(array_pos)
array_speed = np.array(array_speed)

# Create figure with two subplots
plt.figure(figsize=(12, 6))

# Subplot 1: Position vs Crank Angle
plt.subplot(1, 2, 1)
plt.plot(array_angle, array_pos, 'b-')
plt.xlabel('Crank Angle [rad]')
plt.ylabel('Piston Position [m]')
plt.grid(True)
plt.xticks(
    [0, np.pi/2, np.pi, 3*np.pi/2, 2*np.pi],
    ['0', r'$\pi/2$', r'$\pi$', r'$3\pi/2$', r'$2\pi$']
)

# Subplot 2: Speed vs Crank Angle
plt.subplot(1, 2, 2)
plt.plot(array_angle, array_speed, 'r-')
plt.xlabel('Crank Angle [rad]')
plt.ylabel('Piston Speed [m/s]')
plt.grid(True)
plt.xticks(
    [0, np.pi/2, np.pi, 3*np.pi/2, 2*np.pi],
    ['0', r'$\pi/2$', r'$\pi$', r'$3\pi/2$', r'$2\pi$']
)

plt.tight_layout()
plt.show()