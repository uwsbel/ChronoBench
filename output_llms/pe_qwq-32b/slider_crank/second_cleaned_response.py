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
rotation_crank = chrono.ChQuaternionD()
rotation_crank.Q_from_AngAxis(chrono.CH_C_PI_2, chrono.ChVectorD(1, 0, 0))  
mcrank.SetRot(rotation_crank)
mcrank.SetPos(crank_center + chrono.ChVectorD(0, 0, -0.1))
sys.Add(mcrank)

mrod = chrono.ChBodyEasyBox(rod_length, 0.1, 0.1, 1000)
mrod.SetPos(crank_center + chrono.ChVectorD(crank_rad + rod_length/2, 0, 0))
sys.Add(mrod)

mpiston = chrono.ChBodyEasyCylinder(chrono.ChVectorD(0, 1, 0), 0.2, 0.3, 1000)
rotation_piston = chrono.ChQuaternionD()
rotation_piston.Q_from_AngAxis(-chrono.CH_C_PI_2, chrono.ChVectorD(0, 0, 1))  
mpiston.SetRot(rotation_piston)
mpiston.SetPos(crank_center + chrono.ChVectorD(crank_rad + rod_length, 0, 0))
sys.Add(mpiston)

my_motor = chrono.ChLinkMotorRotationSpeed()
my_motor.Initialize(mcrank, mfloor, chrono.ChFrameD(crank_center))
my_angularspeed = chrono.ChFunction_Const(chrono.CH_C_PI)  
my_motor.SetSpeedFunction(my_angularspeed)
sys.Add(my_motor)

mjointA = chrono.ChLinkLockRevolute()
mjointA.Initialize(mrod, mcrank, chrono.ChFrameD(crank_center + chrono.ChVectorD(crank_rad, 0, 0)))
sys.Add(mjointA)

mjointB = chrono.ChLinkLockRevolute()
mjointB.Initialize(mpiston, mrod, chrono.ChFrameD(crank_center + chrono.ChVectorD(crank_rad + rod_length, 0, 0)))
sys.Add(mjointB)


rotation_prismatic = chrono.ChQuaternionD()
rotation_prismatic.Q_from_AngAxis(-chrono.CH_C_PI_2, chrono.ChVectorD(0, 1, 0))  
mjointC = chrono.ChLinkLockPrismatic()
mjointC.Initialize(mpiston, mfloor, chrono.ChFrameD(
    crank_center + chrono.ChVectorD(crank_rad + rod_length, 0, 0),
    rotation_prismatic))
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
time = 0.0

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    sys.DoStepDynamics(1e-3)
    time += 0.001
    
    
    array_time.append(time)
    angle = my_motor.GetAngle()
    array_angle.append(angle)
    pos = mpiston.GetPos().x
    array_pos.append(pos)
    speed = mpiston.GetPos_dt().x
    array_speed.append(speed)
    
    if time > 20:
        break


fig, (ax1, ax2) = plt.subplots(2, 1)
ax1.plot(array_angle, array_pos)
ax1.set_xlabel('Crank Angle (rad)')
ax1.set_ylabel('Position (m)')
ax1.set_xticks([0, np.pi/2, np.pi, 3*np.pi/2, 2*np.pi])
ax1.set_xticklabels(['0', 'π/2', 'π', '3π/2', '2π'])

ax2.plot(array_angle, array_speed)
ax2.set_xlabel('Crank Angle (rad)')
ax2.set_ylabel('Speed (m/s)')
ax2.set_xticks([0, np.pi/2, np.pi, 3*np.pi/2, 2*np.pi])
ax2.set_xticklabels(['0', 'π/2', 'π', '3π/2', '2π'])

plt.tight_layout()
plt.show()