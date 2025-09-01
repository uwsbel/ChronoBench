import pychrono.core as chrono  
import pychrono.irrlicht as chronoirr  
import matplotlib.pyplot as plt  
import numpy as np  


sys = chrono.ChSystemNSC()


crank_center = chrono.ChVector3d(-1, 0.5, 0)  
crank_rad = 0.4  
crank_thick = 0.1  
rod_length = 1.5  


array_time = []
array_angle = []
array_pos = []
array_speed = []


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


sim_time = 0
while vis.Run():
    
    vis.BeginScene()  
    vis.Render()  
    vis.EndScene()  
    sys.DoStepDynamics(1e-3)  

    
    sim_time += 1e-3
    array_time.append(sim_time)
    array_angle.append(my_motor.GetMotorFunction().Get_y(sim_time))
    array_pos.append(mpiston.GetPos().x)
    array_speed.append(mpiston.GetPos_dt().x)

    
    if sim_time >= 20:
        break


fig, axs = plt.subplots(2)
fig.suptitle('Crank Demo')


axs[0].plot(array_angle, array_pos)
axs[0].set_xlabel('Crank Angle [rad]')
axs[0].set_ylabel('Position [m]')
axs[0].set_xticks(np.linspace(0, 2*np.pi, 5))
axs[0].set_xticklabels(['0', 'π/2', 'π', '3π/2', '2π'])


axs[1].plot(array_angle, array_speed)
axs[1].set_xlabel('Crank Angle [rad]')
axs[1].set_ylabel('Speed [m/s]')
axs[1].set_xticks(np.linspace(0, 2*np.pi, 5))
axs[1].set_xticklabels(['0', 'π/2', 'π', '3π/2', '2π'])

plt.show()