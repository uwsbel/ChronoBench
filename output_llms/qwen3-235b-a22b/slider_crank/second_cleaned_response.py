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
    array_time.append(current_time)
    array_angle.append(chrono.CH_PI * current_time)
    array_pos.append(mpiston.GetPos().x())
    array_speed.append(mpiston.GetPos_dt().x())

    if current_time >= 20:
        break


fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))


ax1.plot(array_angle, array_pos, 'b-', label='Position vs Crank Angle')
ax1.set_xlabel('Crank Angle [rad]')
ax1.set_ylabel('Position [m]')
ax1.grid(True)


ax2.plot(array_angle, array_speed, 'r-', label='Speed vs Crank Angle')
ax2.set_xlabel('Crank Angle [rad]')
ax2.set_ylabel('Speed [m/s]')
ax2.grid(True)


xticks = np.pi * np.array([0, 0.5, 1, 1.5, 2, 2.5, 3, 3.5, 4, 4.5, 5, 5.5, 6, 6.5, 7, 7.5, 8, 8.5, 9, 9.5, 10, 10.5, 11, 11.5, 12, 12.5, 13, 13.5, 14, 14.5, 15, 15.5, 16, 16.5, 17, 17.5, 18, 18.5, 19, 19.5, 20])
xtick_labels = ['0', r'$\pi/2$', r'$\pi$', r'$3\pi/2$', r'$2\pi$', r'$5\pi/2$', r'$3\pi$', r'$7\pi/2$', r'$4\pi$', r'$9\pi/2$', r'$5\pi$', r'$11\pi/2$', r'$6\pi$', r'$13\pi/2$', r'$7\pi$', r'$15\pi/2$', r'$8\pi$', r'$17\pi/2$', r'$9\pi$', r'$19\pi/2$', r'$10\pi$', r'$21\pi/2$', r'$11\pi$', r'$23\pi/2$', r'$12\pi$', r'$25\pi/2$', r'$13\pi$', r'$27\pi/2$', r'$14\pi$', r'$29\pi/2$', r'$15\pi$', r'$31\pi/2$', r'$16\pi$', r'$33\pi/2$', r'$17\pi$', r'$35\pi/2$', r'$18\pi$', r'$37\pi/2$', r'$19\pi$', r'$39\pi/2$', r'$20\pi$']

for ax in [ax1, ax2]:
    ax.set_xticks(xticks)
    ax.set_xticklabels(xtick_labels)

plt.tight_layout()
plt.show()