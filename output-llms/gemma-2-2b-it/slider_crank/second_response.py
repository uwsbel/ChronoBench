import pychrono.core as chrono  
import pychrono.irrlicht as chronoirr  
import matplotlib.pyplot as plt  
import numpy as np

# Initialize the Chrono simulation system with non-smooth contact (NSC) method
sys = chrono.ChSystemNSC()

# Define common parameters for the simulation
crank_center = chrono.ChVector3d(-1, 0.5, 0)  
crank_rad = 0.4  
crank_thick = 0.1  
rod_length = 1.5  

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
my_angularspeed = chrono.ChFunctionConst(chrono.CH_PI)  
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

# Create a prismatic joint to connect the piston to the floor, allowing linear motion along the X-axis
mjointC = chrono.ChLinkLockPrismatic()  
mjointC.Initialize(mpiston, mfloor, chrono.ChFramed(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0), chrono.Q_ROTATE_Z_TO_X))  
sys.Add(mjointC)  

# Initialize arrays for storing values to be plotted
array_time = []
array_angle = []
array_pos = []
array_speed = []

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

# Simulation loop
time = 0
while vis.Run():
    # Visualization and time step integration
    vis.BeginScene()  
    vis.Render()  
    vis.EndScene()  
    sys.DoStepDynamics(1e-3)  

    # Collect data during simulation
    time += 1
    array_time.append(time)
    array_angle.append(mcrank.GetAngle())
    array_pos.append(mrod.GetPos())
    array_speed.append(mpiston.GetVelocity())

# Plot the results
plt.figure(figsize=(10, 6))

plt.subplot(2, 1, 1)  
plt.plot(array_time, array_angle)
plt.xlabel('Time (s)')
plt.ylabel('Crankshaft Angle (rad)')
plt.title('Crankshaft Angle vs. Time')
plt.grid(True)

plt.subplot(2, 1, 2)  
plt.plot(array_time, array_pos)
plt.xlabel('Time (s)')
plt.ylabel('Position (m)')
plt.title('Position vs. Time')
plt.grid(True)

plt.tight_layout()  
plt.show()