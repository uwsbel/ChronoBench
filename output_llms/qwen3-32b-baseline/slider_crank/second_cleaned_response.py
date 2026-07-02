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


while vis.Run():
    
    vis.BeginScene()  
    vis.Render()  
    vis.EndScene()  
    sys.DoStepDynamics(1e-3)  

    
    current_time = sys.GetChTime()
    if current_time >= 20.0:
        break

    array_time.append(current_time)
    
    crank_rot = mcrank.GetRot().Q_to_Euler123()
    array_angle.append(crank_rot.z)
    
    piston_pos = mpiston.GetPos().x
    array_pos.append(piston_pos)
    
    piston_vel = mpiston.GetPos_dt().x
    array_speed.append(piston_vel)


plt.figure(figsize=(10, 8))
ax1 = plt.subplot(2, 1, 1)
ax1.plot(array_angle, array_pos, label='Position')
ax1.set_xlabel('Crank Angle [rad]')
ax1.set_ylabel('Position [m]')
ax1.grid(True)

ax2 = plt.subplot(2, 1, 2)
ax2.plot(array_angle, array_speed, label='Speed', color='orange')
ax2.set_xlabel('Crank Angle [rad]')
ax2.set_ylabel('Speed [m/s]')
ax2.grid(True)


import matplotlib.ticker as ticker

def format_angle(val, pos=None):
    
    multiple = (val / np.pi) * 2
    if multiple.is_integer():
        k = int(multiple)
        if k == 0:
            return '0'
        elif k == 1:
            return r'$\frac{\pi}{2}$'
        elif k == 2:
            return r'$\pi$'
        elif k == 3:
            return r'$\frac{3\pi}{2}$'
        elif k == 4:
            return r'$2\pi$'
        else:
            return f'{k/2}π'
    else:
        return f'{val/np.pi:.1f}π'

for ax in [ax1, ax2]:
    ax.xaxis.set_major_locator(ticker.MultipleLocator(base=np.pi/2))
    ax.xaxis.set_major_formatter(ticker.FuncFormatter(format_angle))

plt.tight_layout()
plt.show()