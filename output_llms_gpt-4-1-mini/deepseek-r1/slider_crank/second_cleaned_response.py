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
mrod.SetPos(crank_center + chrono.ChVector3d(crank_rad + rod_length/2, 0, 0))
sys.Add(mrod)


mpiston = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.2, 0.3, 1000)
mpiston.SetPos(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0))
mpiston.SetRot(chrono.Q_ROTATE_Y_TO_X)
sys.Add(mpiston)


my_motor = chrono.ChLinkMotorRotationSpeed()
my_motor.Initialize(mcrank, mfloor, chrono.ChFrameD(crank_center))
my_motor.SetMotorFunction(chrono.ChFunctionConst(chrono.CH_PI))
sys.Add(my_motor)


quat_prismatic = chrono.QuatFromAngleAxis(-chrono.CH_PI_2, chrono.ChVector3d(0, 1, 0))

mjointA = chrono.ChLinkLockRevolute()
mjointA.Initialize(mrod, mcrank, chrono.ChFrameD(crank_center + chrono.ChVector3d(crank_rad, 0, 0)))
sys.Add(mjointA)

mjointB = chrono.ChLinkLockRevolute()
mjointB.Initialize(mpiston, mrod, chrono.ChFrameD(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0)))
sys.Add(mjointB)

mjointC = chrono.ChLinkLockPrismatic()
mjointC.Initialize(mpiston, mfloor, 
                  chrono.ChFrameD(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0), quat_prismatic))
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


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(1e-3)
    
    current_time = sys.GetChTime()
    
    
    array_time.append(current_time)
    array_angle.append(my_motor.GetMotorAngle())
    array_pos.append(mpiston.GetPos().x)
    array_speed.append(mpiston.GetPos_dt().x)
    
    
    if current_time >= 20:
        vis.GetDevice().closeDevice()
        break


plt.figure(figsize=(10, 6))


plt.subplot(2, 1, 1)
plt.plot(array_angle, array_pos, 'b-')
plt.xlabel('Crank Angle [rad]')
plt.ylabel('Position [m]')
plt.title('Position vs Crank Angle')
plt.grid(True)


plt.subplot(2, 1, 2)
plt.plot(array_angle, array_speed, 'r-')
plt.xlabel('Crank Angle [rad]')
plt.ylabel('Speed [m/s]')
plt.title('Speed vs Crank Angle')
plt.grid(True)


xticks = np.arange(0, 2*np.pi + 0.1, np.pi/2)
xtick_labels = ['0', 'π/2', 'π', '3π/2', '2π']
for ax in plt.gcf().axes:
    ax.set_xticks(xticks)
    ax.set_xticklabels(xtick_labels)

plt.tight_layout()
plt.show()