import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import matplotlib.pyplot as plt
import numpy as np


array_time = []
array_angle = []
array_pos = []
array_speed = []


sys = chrono.ChSystemNSC()


crank_center = chrono.ChVectorD(-1, 0.5, 0)
crank_rad = 0.4
crank_thick = 0.1
rod_length = 1.5


mfloor = chrono.ChBodyEasyBox(3, 1, 3, 1000)
mfloor.SetPos(chrono.ChVectorD(0, -0.5, 0))
mfloor.SetBodyFixed(True)
sys.Add(mfloor)


mcrank = chrono.ChBodyEasyCylinder(crank_rad, crank_thick, 1000, True)
mcrank.SetPos(crank_center + chrono.ChVectorD(0, 0, -0.1))
mcrank.SetRot(chrono.Q_ROTATE_Y_TO_Z)
sys.Add(mcrank)


mrod = chrono.ChBodyEasyBox(rod_length, 0.1, 0.1, 1000)
mrod.SetPos(crank_center + chrono.ChVectorD(crank_rad + rod_length / 2, 0, 0))
sys.Add(mrod)


mpiston = chrono.ChBodyEasyCylinder(0.2, 0.3, 1000, True)
mpiston.SetPos(crank_center + chrono.ChVectorD(crank_rad + rod_length, 0, 0))
mpiston.SetRot(chrono.Q_ROTATE_Y_TO_X)
sys.Add(mpiston)


my_motor = chrono.ChLinkMotorRotationSpeed()
my_motor.Initialize(mcrank, mfloor, chrono.ChFrameD(crank_center))
my_angularspeed = chrono.ChFunction_Const(chrono.CH_C_PI)
my_motor.SetSpeedFunction(my_angularspeed)
sys.Add(my_motor)


mjointA = chrono.ChLinkLockRevolute()
mjointA.Initialize(mrod, mcrank, chrono.ChCoordsysD(crank_center + chrono.ChVectorD(crank_rad, 0, 0)))
sys.Add(mjointA)

mjointB = chrono.ChLinkLockRevolute()
mjointB.Initialize(mpiston, mrod, chrono.ChCoordsysD(crank_center + chrono.ChVectorD(crank_rad + rod_length, 0, 0)))
sys.Add(mjointB)

mjointC = chrono.ChLinkLockPrismatic()
mjointC.Initialize(mpiston, mfloor, chrono.ChCoordsysD(crank_center + chrono.ChVectorD(crank_rad + rod_length, 0, 0), chrono.Q_ROTATE_Z_TO_X))
sys.Add(mjointC)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Crank demo')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(1, 1, 3))
vis.AddTypicalLights()


simulation_duration = 20  
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(1e-3)
    
    
    time = sys.GetChTime()
    angle = mcrank.GetRot().Q_to_Euler123().x
    pos = mpiston.GetPos().x
    speed = mpiston.GetPos_dt().x
    
    array_time.append(time)
    array_angle.append(angle)
    array_pos.append(pos)
    array_speed.append(speed)
    
    
    if time >= simulation_duration:
        break


fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 10))


ax1.plot(array_angle, array_pos)
ax1.set_xlabel('Crank Angle [rad]')
ax1.set_ylabel('Position [m]')
ax1.set_title('Piston Position vs Crank Angle')


ax2.plot(array_angle, array_speed)
ax2.set_xlabel('Crank Angle [rad]')
ax2.set_ylabel('Speed [m/s]')
ax2.set_title('Piston Speed vs Crank Angle')


for ax in [ax1, ax2]:
    ax.set_xticks(np.linspace(0, 2*np.pi, 5))
    ax.set_xticklabels(['0', 'π/2', 'π', '3π/2', '2π'])

plt.tight_layout()
plt.show()