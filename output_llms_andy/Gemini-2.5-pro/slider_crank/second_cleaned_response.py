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

mrod.SetPos(crank_center + chrono.ChVector3d(crank_rad + rod_length / 2, 0, 0))
sys.Add(mrod)  


mpiston = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.2, 0.3, 1000)  

mpiston.SetPos(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0))
mpiston.SetRot(chrono.Q_ROTATE_Y_TO_X)  
sys.Add(mpiston)  


my_motor = chrono.ChLinkMotorRotationSpeed()  

my_motor.Initialize(mcrank, mfloor, chrono.ChFramed(crank_center))
my_angularspeed_func = chrono.ChFunctionConst(chrono.CH_PI)  
my_motor.SetMotorFunction(my_angularspeed_func)  
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

vis.AddCamera(chrono.ChVector3d(1, 1, 3), chrono.ChVector3d(0, 0.5, 0))
vis.AddTypicalLights()  


simulation_time_limit = 20.0  

while vis.Run():
    vis.BeginScene()  
    vis.Render()  
    vis.EndScene()  
    
    sys.DoStepDynamics(1e-3)  

    
    current_time = sys.GetChTime()
    array_time.append(current_time)
    
    
    crank_angle = my_motor.GetMotorRot()
    array_angle.append(crank_angle)
    
    
    piston_pos_x = mpiston.GetPos().x
    array_pos.append(piston_pos_x)
    
    
    piston_speed_x = mpiston.GetPosDt().x
    array_speed.append(piston_speed_x)

    
    if current_time >= simulation_time_limit:
        break


if vis.Run():
    vis.GetDevice().closeDevice()




def format_func_pi_ticks(value, tick_number):
    N = int(np.round(2 * value / np.pi)) 
    if N == 0:
        return "0"
    elif N == 1:
        return r"$\pi/2$"
    elif N == -1:
        return r"$-\pi/2$"
    elif N == 2:
        return r"$\pi$"
    elif N == -2:
        return r"$-\pi$"
    elif N % 2 == 0:  
        return r"${}\pi$".format(N // 2)
    else:  
        return r"${}\pi/2$".format(N)

plt.figure(figsize=(12, 8))


plt.subplot(2, 1, 1)
if array_angle: 
    plt.plot(array_angle, array_pos)
    
    ax1 = plt.gca()
    ax1.xaxis.set_major_locator(plt.MultipleLocator(base=np.pi / 2))
    ax1.xaxis.set_major_formatter(plt.FuncFormatter(format_func_pi_ticks))
    plt.xlim([min(array_angle), max(array_angle)])
else:
    plt.text(0.5, 0.5, "No data collected for plotting.", horizontalalignment='center', verticalalignment='center')
plt.title('Piston Position vs. Crank Angle')
plt.xlabel('Crank Angle [rad]')
plt.ylabel('Piston Position [m]')
plt.grid(True)



plt.subplot(2, 1, 2)
if array_angle: 
    plt.plot(array_angle, array_speed, color='orange')
    
    ax2 = plt.gca()
    ax2.xaxis.set_major_locator(plt.MultipleLocator(base=np.pi / 2))
    ax2.xaxis.set_major_formatter(plt.FuncFormatter(format_func_pi_ticks))
    plt.xlim([min(array_angle), max(array_angle)])
else:
    plt.text(0.5, 0.5, "No data collected for plotting.", horizontalalignment='center', verticalalignment='center')
plt.title('Piston Speed vs. Crank Angle')
plt.xlabel('Crank Angle [rad]')
plt.ylabel('Piston Speed [m/s]')
plt.grid(True)

plt.tight_layout()  
plt.show()