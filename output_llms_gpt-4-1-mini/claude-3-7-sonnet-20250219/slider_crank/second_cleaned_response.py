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
mfloor.SetBodyFixed(True)  
sys.Add(mfloor)  


mcrank = chrono.ChBodyEasyCylinder(crank_rad, crank_thick, 1000, chrono.ChVectorD(0, 1, 0))  
mcrank.SetPos(crank_center + chrono.ChVectorD(0, 0, 0))  
mcrank.SetRot(chrono.Q_ROTATE_Y_TO_Z)  
sys.Add(mcrank)  


mrod = chrono.ChBodyEasyBox(rod_length, 0.1, 0.1, 1000)  
mrod.SetPos(crank_center + chrono.ChVectorD(crank_rad + rod_length / 2, 0, 0))  
sys.Add(mrod)  


mpiston = chrono.ChBodyEasyCylinder(0.2, 0.3, 1000, chrono.ChVectorD(1, 0, 0))  
mpiston.SetPos(crank_center + chrono.ChVectorD(crank_rad + rod_length, 0, 0))  
sys.Add(mpiston)  


my_motor = chrono.ChLinkMotorRotationSpeed()  
my_motor.Initialize(mcrank, mfloor, chrono.ChFrame(crank_center))  
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
vis.AddCamera(chrono.ChVectorD(1, 1, 3), chrono.ChVectorD(0, 1, 0))  
vis.AddTypicalLights()  


array_time = []
array_angle = []
array_pos = []
array_speed = []


sim_end_time = 20.0  
sim_time = 0.0
time_step = 1e-3  


while vis.Run() and sim_time < sim_end_time:
    
    vis.BeginScene()  
    vis.Render()  
    vis.EndScene()  
    
    
    sys.DoStepDynamics(time_step)  
    sim_time += time_step
    
    
    crank_angle = mcrank.GetRot().Q_to_Euler123().y  
    piston_pos = mpiston.GetPos().x  
    piston_speed = mpiston.GetPos_dt().x  
    
    
    array_time.append(sim_time)
    array_angle.append(crank_angle)
    array_pos.append(piston_pos)
    array_speed.append(piston_speed)


array_time = np.array(array_time)
array_angle = np.array(array_angle)
array_pos = np.array(array_pos)
array_speed = np.array(array_speed)


plt.figure(figsize=(12, 10))


plt.subplot(2, 1, 1)
plt.plot(array_angle, array_pos, 'b-')
plt.title('Piston Position vs Crank Angle')
plt.xlabel('Crank Angle [rad]')
plt.ylabel('Piston Position [m]')


plt.xticks(np.arange(0, 2*np.pi+0.1, np.pi/2), 
           ['0', 'π/2', 'π', '3π/2', '2π'])


plt.subplot(2, 1, 2)
plt.plot(array_angle, array_speed, 'r-')
plt.title('Piston Speed vs Crank Angle')
plt.xlabel('Crank Angle [rad]')
plt.ylabel('Piston Speed [m/s]')


plt.xticks(np.arange(0, 2*np.pi+0.1, np.pi/2), 
           ['0', 'π/2', 'π', '3π/2', '2π'])

plt.tight_layout()
plt.show()