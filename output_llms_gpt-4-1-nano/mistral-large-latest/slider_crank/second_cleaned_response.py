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
mcrank.SetPos(crank_center)  
mcrank.SetRot(chrono.Q_ROTATE_Y_TO_Z)  
sys.Add(mcrank)  


mrod = chrono.ChBodyEasyBox(rod_length, 0.1, 0.1, 1000)  
mrod.SetPos(crank_center + chrono.ChVector3d(crank_rad, 0, 0))  
sys.Add(mrod)  


mpiston = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.2, 0.3, 1000)  
mpiston.SetPos(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0))  
mpiston.SetRot(chrono.Q_ROTATE_Y_TO_X)  
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
mjointC.Initialize(mpiston, mfloor, chrono.ChFrameD(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0), chrono.Q_ROTATE_Z_TO_X))  
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
vis.AddLogo(chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')  
vis.AddSkyBox()  
vis.AddCamera(chrono.ChVector3d(1, 1, 3), chrono.ChVector3d(0, 1, 0))  
vis.AddTypicalLights()  


simulation_time = 0
step_size = 1e-3  
max_sim_time = 20  

while vis.Run() and simulation_time < max_sim_time:
    
    vis.BeginScene()  
    vis.Render()  
    vis.EndScene()  

    
    time = sys.GetChTime()
    angle = my_motor.GetMotorRot()
    pos = mpiston.GetPos().x
    speed = mpiston.GetPos_dt().x

    array_time.append(time)
    array_angle.append(angle)
    array_pos.append(pos)
    array_speed.append(speed)

    
    sys.DoStepDynamics(step_size)
    simulation_time += step_size


fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)


ax1.plot(array_angle, array_pos)
ax1.set_ylabel('Position [m]')
ax1.set_title('Position vs. Crank Angle')
ax1.xaxis.set_major_locator(plt.MultipleLocator(np.pi / 2))
ax1.xaxis.set_minor_locator(plt.MultipleLocator(np.pi / 4))
ax1.xaxis.set_major_formatter(plt.FuncFormatter(lambda val, pos: '{:.0f}$\pi$'.format(val / np.pi) if val != 0 else '0'))


ax2.plot(array_angle, array_speed)
ax2.set_ylabel('Speed [m/s]')
ax2.set_xlabel('Crank Angle [rad]')
ax2.set_title('Speed vs. Crank Angle')
ax2.xaxis.set_major_locator(plt.MultipleLocator(np.pi / 2))
ax2.xaxis.set_minor_locator(plt.MultipleLocator(np.pi / 4))
ax2.xaxis.set_major_formatter(plt.FuncFormatter(lambda val, pos: '{:.0f}$\pi$'.format(val / np.pi) if val != 0 else '0'))

plt.show()