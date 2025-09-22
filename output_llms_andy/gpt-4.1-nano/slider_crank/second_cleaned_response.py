import pychrono.core as chrono  
import pychrono.irrlicht as chronoirr  
import matplotlib.pyplot as plt  
import numpy as np  


sys = chrono.ChSystemNSC()


crank_center = chrono.ChVector3d(-1, 0.5, 0)  
crank_rad = 0.4  
crank_thick = 0.1  
rod_length = 1.5  


mfloor = chrono.ChBodyEasyBox(3, 1, 3, 1000)  
mfloor.SetPos(chrono.ChVector3d(0, -0.5, 0))  
mfloor.SetFixed(True)  
sys.Add(mfloor)  


mcrank = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, crank_rad, crank_thick, 1000)  
mcrank.SetPos(crank_center + chrono.ChVector3d(0, 0, -0.1))  

mcrank.SetRot(chrono.Q_from_AngZ(0))  
sys.Add(mcrank)  


mrod = chrono.ChBodyEasyBox(rod_length, 0.1, 0.1, 1000)  
mrod.SetPos(crank_center + chrono.ChVector3d(crank_rad + rod_length / 2, 0, 0))
sys.Add(mrod)


mpiston = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.2, 0.3, 1000)
mpiston.SetPos(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0))

mpiston.SetRot(chrono.Q_from_AngX(0))
sys.Add(mpiston)


my_motor = chrono.ChLinkMotorRotationSpeed()
my_motor.Initialize(mcrank, mfloor, chrono.ChFrameD(crank_center))
my_angularspeed = chrono.ChFunction_Const(chrono.CH_C_PI)  
my_motor.SetMotorFunction(my_angularspeed)
sys.Add(my_motor)


mjointA = chrono.ChLinkLockRevolute()
mjointA.Initialize(mrod, mcrank, chrono.ChFrameD(crank_center + chrono.ChVector3d(crank_rad, 0, 0)))
sys.Add(mjointA)


mjointB = chrono.ChLinkLockRevolute()
mjointB.Initialize(mpiston, mrod, chrono.ChFrameD(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0)))
sys.Add(mjointB)


mjointC = chrono.ChLinkLockPrismatic()

frame_for_prismatic = chrono.ChFrameD(
    chrono.ChVector3d(crank_rad + rod_length, 0, 0),
    chrono.Q_from_AngX(0)
)
mjointC.Initialize(mpiston, mfloor, frame_for_prismatic)
sys.Add(mjointC)


array_time = []
array_angle = []
array_pos = []
array_speed = []


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Crank demo')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()

vis.AddCamera(chrono.ChVector3d(1, 1, 3), chrono.ChVector3d(0, 1, 0))
vis.AddTypicalLights()


time_end = 20.0
dt = 1e-3
time = 0.0

while vis.Run():
    
    
    crank_rot = mcrank.GetRot()
    
    
    angle_z = crank_rot.Get_AngleZ()
    
    crank_pos = mcrank.GetPos()
    
    crank_speed = mcrank.GetPos_dt().Length()

    
    array_time.append(time)
    array_angle.append(angle_z)
    array_pos.append(crank_pos.x)
    array_speed.append(crank_speed)

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    sys.DoStepDynamics(dt)
    time += dt

    
    if time >= time_end:
        break



angle_array = np.array(array_angle)
pos_array = np.array(array_pos)
speed_array = np.array(array_speed)


plt.figure(figsize=(10, 8))
plt.subplot(2, 1, 1)
plt.plot(angle_array, pos_array)
plt.xlabel('Crank angle [rad]')
plt.ylabel('Position [m]')
plt.title('Crank Position vs. Angle')
plt.grid(True)


plt.subplot(2, 1, 2)
plt.plot(angle_array, speed_array)
plt.xlabel('Crank angle [rad]')
plt.ylabel('Speed [m/s]')
plt.title('Crank Speed vs. Angle')
plt.grid(True)


ticks = np.array([0, np.pi/2, np.pi, 3*np.pi/2, 2*np.pi])
tick_labels = ['0', 'π/2', 'π', '3π/2', '2π']
for ax in plt.gcf().axes:
    ax.set_xticks(ticks)
    ax.set_xticklabels(tick_labels)

plt.tight_layout()
plt.show()