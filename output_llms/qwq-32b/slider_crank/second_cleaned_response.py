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


mcrank = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, crank_rad, crank_thick, 1000)  
mcrank.SetPos(crank_center + chrono.ChVectorD(0, 0, -0.1))  
mcrank.SetRot(chrono.Q_ROTATE_Y_TO_Z)  
sys.Add(mcrank)  


crank_end = crank_center + chrono.ChVectorD(0, 0, crank_thick / 2)


mrod = chrono.ChBodyEasyBox(rod_length, 0.1, 0.1, 1000)  
rod_center = crank_end + chrono.ChVectorD(rod_length / 2, 0, 0)
mrod.SetPos(rod_center)  
sys.Add(mrod)  


piston_pos = crank_end + chrono.ChVectorD(rod_length, 0, 0)
mpiston = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.2, 0.3, 1000)  
mpiston.SetPos(piston_pos)  
mpiston.SetRot(chrono.Q_ROTATE_Y_TO_X)  
sys.Add(mpiston)  


my_motor = chrono.ChLinkMotorRotationSpeed()  
my_motor.Initialize(mcrank, mfloor, chrono.ChFrameD(crank_center))  
my_angularspeed = chrono.ChFunctionConst(chrono.CH_PI)  
my_motor.SetMotorFunction(my_angularspeed)  
sys.Add(my_motor)  


mjointA = chrono.ChLinkLockRevolute()  
mjointA.Initialize(mrod, mcrank, chrono.ChFrameD(crank_end))  
sys.Add(mjointA)  


mjointB = chrono.ChLinkLockRevolute()  
mjointB.Initialize(mpiston, mrod, chrono.ChFrameD(piston_pos))  
sys.Add(mjointB)  


mjointC = chrono.ChLinkLockPrismatic()  
mjointC.Initialize(mpiston, mfloor, chrono.ChFrameD(piston_pos, chrono.Q_ROTATE_Z_TO_X))  
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
vis.AddCamera(chrono.ChVectorD(1, 1, 3), chrono.ChVectorD(0, 1, 0))  
vis.AddTypicalLights()  


while vis.Run():
    
    vis.BeginScene()  
    vis.Render()  
    vis.EndScene()  
    
    current_time = sys.GetChTime()
    array_time.append(current_time)
    array_angle.append(chrono.CH_PI * current_time)
    array_pos.append(mpiston.GetPos().x)
    array_speed.append(mpiston.GetPos_dt().x)
    
    if current_time >= 20:
        break
    
    sys.DoStepDynamics(1e-3)  


plt.figure(figsize=(12, 8))


plt.subplot(2, 1, 1)
plt.plot(array_angle, array_pos, label='Position')
plt.xlabel('Crank Angle [rad]')
plt.ylabel('Position [m]')
plt.xticks([0, np.pi/2, np.pi, 3*np.pi/2, 2*np.pi],
           ['0', r'$\pi/2$', r'$\pi$', r'$3\pi/2$', r'$2\pi$'])
plt.grid(True)


plt.subplot(2, 1, 2)
plt.plot(array_angle, array_speed, label='Speed')
plt.xlabel('Crank Angle [rad]')
plt.ylabel('Speed [m/s]')
plt.xticks([0, np.pi/2, np.pi, 3*np.pi/2, 2*np.pi],
           ['0', r'$\pi/2$', r'$\pi$', r'$3\pi/2$', r'$2\pi$'])
plt.grid(True)

plt.tight_layout()
plt.show()