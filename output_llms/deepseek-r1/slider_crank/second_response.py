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

# Create floor (fixed)
mfloor = chrono.ChBodyEasyBox(3, 1, 3, 1000)
mfloor.SetPos(chrono.ChVector3d(0, -0.5, 0))
mfloor.SetFixed(True)
sys.Add(mfloor)

# Create crank (cylinder)
mcrank = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, crank_rad, crank_thick, 1000)
mcrank.SetPos(crank_center + chrono.ChVector3d(0, 0, -0.1))
mcrank.SetRot(chrono.Q_ROTATE_Y_TO_Z)  # Align cylinder axis with Z
sys.Add(mcrank)

# Create connecting rod (box)
mrod = chrono.ChBodyEasyBox(rod_length, 0.1, 0.1, 1000)
mrod.SetPos(crank_center + chrono.ChVector3d(crank_rad + rod_length / 2, 0, 0))
sys.Add(mrod)

# Create piston (cylinder)
mpiston = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.2, 0.3, 1000)
mpiston.SetPos(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0))
mpiston.SetRot(chrono.Q_ROTATE_Y_TO_X)  # Align cylinder axis with X
sys.Add(mpiston)

# Create crankshaft motor
my_motor = chrono.ChLinkMotorRotationSpeed()
my_motor.Initialize(mcrank, mfloor, chrono.ChFrameD(crank_center))  # Fixed ChFrameD
my_angularspeed = chrono.ChFunctionConst(chrono.CH_PI)  # π rad/s constant speed
my_motor.SetMotorFunction(my_angularspeed)
sys.Add(my_motor)

# Create crank-rod joint (revolute)
mjointA = chrono.ChLinkLockRevolute()
mjointA.Initialize(mrod, mcrank, chrono.ChFrameD(crank_center + chrono.ChVector3d(crank_rad, 0, 0)))  # Fixed
sys.Add(mjointA)

# Create rod-piston joint (revolute)
mjointB = chrono.ChLinkLockRevolute()
mjointB.Initialize(mpiston, mrod, chrono.ChFrameD(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0)))  # Fixed
sys.Add(mjointB)

# Create piston guide (prismatic joint - fixed to move along X)
mjointC = chrono.ChLinkLockPrismatic()
# Removed rotation parameter to default to X-axis motion
mjointC.Initialize(mpiston, mfloor, chrono.ChFrameD(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0)))
sys.Add(mjointC)

# Setup visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Crank Mechanism Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1, 1, 3), chrono.ChVector3d(0, 1, 0))
vis.AddTypicalLights()

# Data collection arrays
array_time = []
array_angle = []
array_pos = []
array_speed = []

# Simulation parameters
sim_time = 0
end_time = 20  # 20-second simulation

# Run simulation
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    # Advance simulation by 1 ms
    sys.DoStepDynamics(1e-3)
    
    # Update simulation time
    sim_time = sys.GetChTime()
    
    # Stop simulation after 20 seconds
    if sim_time >= end_time:
        break
    
    # Collect data
    crank_angle = my_motor.GetMotorRot()  # Current crank angle [rad]
    piston_pos = mpiston.GetPos().x       # Piston X-position [m]
    piston_speed = mpiston.GetPosDt().x   # Piston X-velocity [m/s]
    
    # Store data
    array_time.append(sim_time)
    array_angle.append(crank_angle)
    array_pos.append(piston_pos)
    array_speed.append(piston_speed)

# Create plots after simulation
plt.figure(figsize=(12, 5))

# Position vs Angle plot
plt.subplot(1, 2, 1)
plt.plot(array_angle, array_pos, 'b-')
plt.xlabel('Crank Angle [rad]')
plt.ylabel('Piston Position [m]')
plt.title('Position vs Crank Angle')
plt.grid(True)
plt.xticks(
    [0, np.pi/2, np.pi, 3*np.pi/2, 2*np.pi],
    ['0', 'π/2', 'π', '3π/2', '2π']
)

# Speed vs Angle plot
plt.subplot(1, 2, 2)
plt.plot(array_angle, array_speed, 'r-')
plt.xlabel('Crank Angle [rad]')
plt.ylabel('Piston Speed [m/s]')
plt.title('Speed vs Crank Angle')
plt.grid(True)
plt.xticks(
    [0, np.pi/2, np.pi, 3*np.pi/2, 2*np.pi],
    ['0', 'π/2', 'π', '3π/2', '2π']
)

plt.tight_layout()
plt.show()