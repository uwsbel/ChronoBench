import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import matplotlib.pyplot as plt
import numpy as np


sys = chrono.ChSystemNSC()


crank_center = chrono.ChVector3d(-1, 0.5, 0)
crank_rad = 0.4
crank_thick = 0.1
rod_length = 1.5


array_time = [0.0]
array_angle = [0.0]
array_pos = [crank_center.x + crank_rad + rod_length]
array_speed = [0.0]


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
my_motor.Initialize(mcrank, mfloor, chrono.ChFrameD(crank_center))
my_angularspeed = chrono.ChFunctionConst(chrono.CH_PI)
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


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Crank demo')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1, 1, 3), chrono.ChVector3d(0, 1, 0))
vis.AddTypicalLights()


simulation_time = 0.0
total_time = 0.0
time_step = 1e-3

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    
    simulation_time = sys.GetChTime()
    if simulation_time <= 20.0:
        crank_angle = mcrank.GetRot().Q_toAngleAxis()[0]
        piston_pos = mpiston.GetPos().x
        piston_speed = mpiston.GetPos_dt().x
        
        array_time.append(simulation_time)
        array_angle.append(crank_angle)
        array_pos.append(piston_pos)
        array_speed.append(piston_speed)
    else:
        break
    
    sys.DoStepDynamics(time_step)


plt.figure(figsize=(12, 8))


plt.subplot(2, 1, 1)
plt.plot(array_angle, array_pos, label='Position [m]')
plt.xlabel('Crank Angle [rad]')
plt.ylabel('Position [m]')
plt.title('Piston Position vs Crank Angle')
plt.grid(True)


plt.subplot(2, 1, 2)
plt.plot(array_angle, array_speed, label='Speed [m/s]', color='red')
plt.xlabel('Crank Angle [rad]')
plt.ylabel('Speed [m/s]')
plt.title('Piston Speed vs Crank Angle')
plt.grid(True)


for ax in plt.gcf().axes:
    ax.set_xticks(np.arange(0, 2 * np.pi + 0.1, np.pi/2))
    ax.set_xticklabels(['0', '$\\pi/2$', '$\\pi$', '$3\\pi/2$', '$2\\pi$'])

plt.tight_layout()
plt.show()