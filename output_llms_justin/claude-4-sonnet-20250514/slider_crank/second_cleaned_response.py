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
mcrank.SetRot(chrono.Q_ROTATE_Y_TO_Z)  
sys.Add(mcrank)  


mrod = chrono.ChBodyEasyBox(rod_length, 0.1, 0.1, 1000)  
mrod.SetPos(crank_center + chrono.ChVector3d(crank_rad + rod_length / 2, 0, 0))  
sys.Add(mrod)  


mpiston = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.2, 0.3, 1000)  
mpiston.SetPos(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0))  
mpiston.SetRot(chrono.Q_ROTATE_Y_TO_X)  
sys.Add(mpiston)  


my_motor = chrono.ChLinkMotorRotationSpeed()  
my_motor.Initialize(mcrank, mfloor, chrono.ChFramed(crank_center))  
my_angularspeed = chrono.ChFunctionConst(chrono.CH_PI)  
my_motor.SetMotorFunction(my_angularspeed)  
sys.Add(my_motor)  


mjointA = chrono.ChLinkLockRevolute()  
mjointA.Initialize(mrod, mcrank, chrono.ChFramed(crank_center + chrono.ChVector3d(crank_rad, 0, 0)))  
sys.Add(mjointA)  


mjointB = chrono.ChLinkLockRevolute()  
mjointB.Initialize(mpiston, mrod, chrono.ChFramed(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0)))  
sys.Add(mjointB)  


mjointC = chrono.ChLinkLockPrismatic()  
mjointC.Initialize(mpiston, mfloor, chrono.ChFramed(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0), chrono.Q_ROTATE_Z_TO_X))  
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


time_step = 1e-3  
time_end = 20.0  


while vis.Run():
    
    current_time = sys.GetChTime()
    
    
    if current_time >= time_end:
        break
    
    
    
    crank_rot = mcrank.GetRot()
    crank_angle = crank_rot.GetCardanAnglesZYX().z  
    
    
    piston_pos = mpiston.GetPos().x
    
    
    piston_vel = mpiston.GetPosDt().x
    
    
    array_time.append(current_time)
    array_angle.append(crank_angle)
    array_pos.append(piston_pos)
    array_speed.append(piston_vel)
    
    
    vis.BeginScene()  
    vis.Render()  
    vis.EndScene()  
    sys.DoStepDynamics(time_step)  



array_angle = np.array(array_angle)
array_pos = np.array(array_pos)
array_speed = np.array(array_speed)


fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))


ax1.plot(array_angle, array_pos, 'b-', linewidth=2)
ax1.set_xlabel('Crank Angle [rad]')
ax1.set_ylabel('Piston Position [m]')
ax1.set_title('Piston Position vs Crank Angle')
ax1.grid(True)


pi_ticks = np.arange(0, 2*np.pi*np.ceil(np.max(array_angle)/(2*np.pi)) + np.pi/2, np.pi/2)
ax1.set_xticks(pi_ticks)
ax1.set_xticklabels(['0', 'π/2', 'π', '3π/2', '2π', '5π/2', '3π', '7π/2', '4π', '9π/2', '5π', '11π/2', '6π'][:len(pi_ticks)])


ax2.plot(array_angle, array_speed, 'r-', linewidth=2)
ax2.set_xlabel('Crank Angle [rad]')
ax2.set_ylabel('Piston Speed [m/s]')
ax2.set_title('Piston Speed vs Crank Angle')
ax2.grid(True)


ax2.set_xticks(pi_ticks)
ax2.set_xticklabels(['0', 'π/2', 'π', '3π/2', '2π', '5π/2', '3π', '7π/2', '4π', '9π/2', '5π', '11π/2', '6π'][:len(pi_ticks)])


plt.tight_layout()
plt.show()