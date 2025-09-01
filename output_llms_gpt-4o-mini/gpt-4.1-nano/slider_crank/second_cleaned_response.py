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


mcrank.SetRot(chrono.Q_from_AngY(chrono.C_PI / 2))
sys.Add(mcrank)


mrod = chrono.ChBodyEasyBox(rod_length, 0.1, 0.1, 1000)  
mrod.SetPos(crank_center + chrono.ChVector3d(crank_rad + rod_length/2, 0, 0))
sys.Add(mrod)


mpiston = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.2, 0.3, 1000)
mpiston.SetPos(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0))

mpiston.SetRot(chrono.Q_from_AngX(chrono.C_PI / 2))
sys.Add(mpiston)


my_motor = chrono.ChLinkMotorRotationSpeed()
my_motor.Initialize(mcrank, mfloor, chrono.ChFrameD(crank_center))
my_angularspeed = chrono.ChFunction_Const(chrono.C_PI)  
my_motor.SetMotorFunction(my_angularspeed)
sys.Add(my_motor)


mjointA = chrono.ChLinkLockRevolute()
mjointA.Initialize(mrod, mcrank, chrono.ChFrameD(crank_center + chrono.ChVector3d(crank_rad, 0, 0)))
sys.Add(mjointA)


mjointB = chrono.ChLinkLockRevolute()
mjointB.Initialize(mpiston, mrod, chrono.ChFrameD(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0)))
sys.Add(mjointB)



mjointC = chrono.ChLinkLockPrismatic()
joint_frame = chrono.ChFrameD(
    chrono.ChVector3d(crank_rad + rod_length, 0, 0),
    chrono.Q_from_AngX(0)  
)
mjointC.Initialize(mpiston, mfloor, joint_frame)
sys.Add(mjointC)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Crank demo')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1, 1, 3), chrono.ChVector3d(0, 1, 0))
vis.AddTypicalLights()


array_time = []
array_angle = []
array_pos = []
array_speed = []


t_end = 20.0  
dt = 1e-3  


time = 0.0
while vis.Run():
    
    
    crank_rot = mcrank.GetRot()
    
    
    
    
    
    angle = 2 * np.arccos(crank_rot.w)
    
    if crank_rot.y < 0:
        angle = -angle

    
    piston_pos_x = mpiston.GetPos().x()

    
    piston_vel_x = mpiston.GetPos_dt().x()

    
    array_time.append(time)
    array_angle.append(angle)
    array_pos.append(piston_pos_x)
    array_speed.append(piston_vel_x)

    
    if time >= t_end:
        break

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    sys.DoStepDynamics(dt)
    time += dt



array_time_np = np.array(array_time)
array_angle_np = np.array(array_angle)
array_pos_np = np.array(array_pos)
array_speed_np = np.array(array_speed)


fig, axs = plt.subplots(2, 1, figsize=(10, 8))


axs[0]..plot(array_angle_np, array_pos_np)
axs[0].set_xlabel('Crank angle [rad]')
axs[0].set_ylabel('Position [m]')
axs[0].set_title('Piston position vs. crank angle')
axs[0].grid()


axs[1].plot(array_angle_np, array_speed_np)
axs[1].set_xlabel('Crank angle [rad]')
axs[1].set_ylabel('Speed [m/s]')
axs[1].set_title('Piston speed vs. crank angle')
axs[1].grid()


ticks = np.array([0, np.pi/2, np.pi, 3*np.pi/2, 2*np.pi])
tick_labels = ['0', 'π/2', 'π', '3π/2', '2π']

for ax in axs:
    ax.set_xticks(ticks)
    ax.set_xticklabels(tick_labels)

plt.tight_layout()
plt.show()