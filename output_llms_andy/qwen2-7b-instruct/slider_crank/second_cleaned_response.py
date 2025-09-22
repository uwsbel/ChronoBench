import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import matplotlib.pyplot as plt
import numpy as np


sys = chrono.ChSystemNSC()


crank_center = chrono.ChVectorD(-1, 0.5, 0)
crank_rad = 0.4
crank_thick = 0.1
rod_length = 1.5


mfloor = chrono.ChBodyEasyBox(3, 1, 3, 1000)
mfloor.SetPos(chrono.ChVectorD(0, -0.5, 0))
mfloor.SetFixed(True)
sys.Add(mfloor)


mcrank = chrono.ChBodyEasyCylinder(chrono.ChVectorD(0, 1, 0), crank_rad, crank_thick, 1000)
mcrank.SetPos(crank_center + chrono.ChVectorD(0, 0, -0.1))
mcrank.SetRot(chrono.Q_ROTATE_Y_TO_Z)
sys.Add(mcrank)


mrod = chrono.ChBodyEasyBox(rod_length, 0.1, 0.1, 1000)
mrod.SetPos(crank_center + chrono.ChVectorD(crank_rad + rod_length / 2, 0, 0))
sys.Add(mrod)


mpiston = chrono.ChBodyEasyCylinder(chrono.ChVectorD(0, 1, 0), 0.2, 0.3, 1000)
mpiston.SetPos(crank_center + chrono.ChVectorD(crank_rad + rod_length, 0, 0))
mpiston.SetRot(chrono.Q_ROTATE_Y_TO_X)
sys.Add(mpiston)


my_motor = chrono.ChLinkMotorRotationSpeed()
my_motor.Initialize(mcrank, mfloor, chrono.ChFramed(crank_center))
my_angularspeed = chrono.ChFunction_Const(chrono.CH_COS(chrono.CH_PI / 2))
my_motor.SetMotorFunction(my_angularspeed)
sys.Add(my_motor)


mjointA = chrono.ChLinkLockRevolute()
mjointA.Initialize(mrod, mcrank, chrono.ChFramed(crank_center + chrono.ChVectorD(crank_rad, 0, 0)))
sys.Add(mjointA)


mjointB = chrono.ChLinkLockRevolute()
mjointB.Initialize(mpiston, mrod, chrono.ChFramed(crank_center + chrono.ChVectorD(crank_rad + rod_length, 0, 0)))
sys.Add(mjointB)


mjointC = chrono.ChLinkLockPrismatic()
mjointC.Initialize(mpiston, mfloor, chrono.ChFramed(crank_center + chrono.ChVectorD(crank_rad + rod_length, 0, 0), chrono.Q_ROTATE_Z_TO_X))
sys.Add(mjointC)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Crank demo')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(1, 1, 3), chrono.ChVectorD(0, 1, 0))
vis.AddTypicalLights()


array_time = []
array_angle = []
array_pos = []
array_speed = []


time_step = 1e-3
total_time = 20
time = 0

while time < total_time:
    sys.DoStepDynamics(time_step)
    time += time_step

    
    array_time.append(time)
    array_angle.append(chrono.GetRad(my_motor.GetMotorSpeed()))
    array_pos.append(mcrank.GetPos().x)
    array_speed.append(chrono.GetRad(my_motor.GetMotorSpeed()) * time_step)


plt.figure(figsize=(12, 6))


plt.subplot(1, 2, 1)
plt.plot(array_angle, array_pos)
plt.title('Position vs. Angle')
plt.xlabel('Angle [rad]')
plt.ylabel('Position [m]')
plt.xticks(np.arange(0, 2 * np.pi + np.pi / 2, np.pi / 2))


plt.subplot(1, 2, 2)
plt.plot(array_angle, array_speed)
plt.title('Speed vs. Angle')
plt.xlabel('Angle [rad]')
plt.ylabel('Speed [m/s]')
plt.xticks(np.arange(0, 2 * np.pi + np.pi / 2, np.pi / 2))

plt.tight_layout()
plt.show()