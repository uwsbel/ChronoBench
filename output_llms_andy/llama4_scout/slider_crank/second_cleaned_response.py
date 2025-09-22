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
mrod.SetPos(crank_center + chrono.ChVector3d(crank_rad, 0, 0))  
sys.Add(mrod)  


mpiston = chrono.ChBodyEasyCylinder(chrono.ChAxis_X, 0.2, 0.3, 1000)  
mpiston.SetPos(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0))  
mpiston.SetRot(chrono.QROTATE_X_TO_Z)  
sys.Add(mpiston)  


my_motor = chrono.ChLinkMotorRotationSpeed()  
my_motor.Initialize(mcrank, mfloor, chrono.ChFramed(crank_center))  
my_angularspeed = chrono.ChFunctionConst(chrono.CH_PI)  
my_motor.SetMotorFunction(my_angularspeed)  
sys.Add(my_motor)  


mjointA = chrono.ChLinkLockRevolute()  
mjointA.Initialize(mcrank, mrod, chrono.ChFramed(crank_center + chrono.ChVector3d(crank_rad, 0, 0)))  
sys.Add(mjointA)  


mjointB = chrono.ChLinkLockRevolute()  
mjointB.Initialize(mrod, mpiston, chrono.ChFramed(crank_center + chrono.ChVector3d(crank_rad + rod_length / 2, 0, 0)))  
sys.Add(mjointB)  


mjointC = chrono.ChLinkLockPrismatic()  
mjointC.Initialize(mpiston, mfloor, chrono.ChFramed(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0), chrono.Q_ROTATE_Z_TO_X))  
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


t = 0
while vis.Run() and t < 20:  
    vis.BeginScene()  
    vis.Render()  
    vis.EndScene()  

    
    crank_angle = my_motor.GetMotorAngle()

    
    piston_pos = mpiston.GetPos().x()

    
    piston_speed = mpiston.GetPos_dt().x()

    
    array_time.append(t)
    array_angle.append(crank_angle)
    array_pos.append(piston_pos)
    array_speed.append(piston_speed)

    
    sys.DoStepDynamics(1e-3)
    t += 1e-3


vis.Close()


fig, axs = plt.subplots(2, figsize=(10, 6))


axs[0].plot(np.array(array_angle), np.array(array_pos))
axs[0].set_title('Position vs. Crank Angle')
axs[0].set_xlabel('Crank Angle [rad]')
axs[0].set_ylabel('Position [m]')
ticks = np.linspace(0, 2 * np.pi, 5)
axs[0].set_xticks(ticks)
axs[0].set_xticklabels([f'{tick:.1f}\n({tick / np.pi:.1f}$\pi$)' for tick in ticks])


axs[1].plot(np.array(array_angle), np.array(array_speed))
axs[1].set_title('Speed vs. Crank Angle')
axs[1].set_xlabel('Crank Angle [rad]')
axs[1].set_ylabel('Speed [m/s]')
axs[1].set_xticks(ticks)
axs[1].set_xticklabels([f'{tick:.1f}\n({tick / np.pi:.1f}$\pi$)' for tick in ticks])


fig.tight_layout()

plt.show()