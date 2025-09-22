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


simulation_time = 20.0  
current_time = 0.0
time_step = 1e-3  


while vis.Run() and current_time < simulation_time:
    
    vis.BeginScene()  
    vis.Render()  
    vis.EndScene()  

    
    crank_angle = my_motor.GetMotorRot()  

    
    piston_pos = mpiston.GetPos().x  
    piston_speed = mpiston.GetPos_dt().x  

    
    array_time.append(current_time)
    array_angle.append(crank_angle)
    array_pos.append(piston_pos)
    array_speed.append(piston_speed)

    
    sys.DoStepDynamics(time_step)
    current_time += time_step


vis.Close()


plt.figure(figsize=(12, 6))


plt.subplot(1, 2, 1)
plt.plot(array_angle, array_pos)
plt.title('Position vs. Crank Angle')
plt.xlabel('Crank Angle [rad]')
plt.ylabel('Position [m]')
plt.grid(True)


plt.subplot(1, 2, 2)
plt.plot(array_angle, array_speed)
plt.title('Speed vs. Crank Angle')
plt.xlabel('Crank Angle [rad]')
plt.ylabel('Speed [m/s]')
plt.grid(True)


for ax in plt.gcf().get_axes():
    ax.xaxis.set_major_locator(plt.MultipleLocator(np.pi/2))
    ax.xaxis.set_major_formatter(plt.FuncFormatter(
        lambda val, pos: '{:.0g}π'.format(val/np.pi) if val != 0 else '0'
    ))

plt.tight_layout()
plt.show()