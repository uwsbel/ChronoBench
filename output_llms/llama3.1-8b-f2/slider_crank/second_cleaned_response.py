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
mcrank.SetRot(chrono.QuatFromAngleAxis(-np.pi / 2, chrono.ChVector3d(1, 0, 0)))  
sys.Add(mcrank)  


mrod = chrono.ChBodyEasyBox(rod_length, 0.1, 0.1, 1000)  
mrod.SetPos(crank_center + chrono.ChVector3d(crank_rad + rod_length / 2, 0, 0))  
sys.Add(mrod)  


mpiston = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.2, 0.3, 1000)  
mpiston.SetPos(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0))  
mpiston.SetRot(chrono.QuatFromAngleAxis(np.pi / 2, chrono.ChVector3d(1, 0, 0)))  
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
mjointC.Initialize(mpiston, mfloor, chrono.ChFramed(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0), chrono.QuatFromAngleAxis(np.pi / 2, chrono.ChVector3d(1, 0, 0))))  
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


step_size = 1e-3


tend = 20


t = 0


render_step_size = 1.0 / 50  


render_steps = int(render_step_size / step_size)


vis.BeginScene()
vis.Render()
vis.EndScene()


step_number = 0


while vis.Run():
    time = sys.GetChTime()

    
    array_time.append(time)
    array_angle.append(mcrank.GetAngleZ())
    array_pos.append(mpiston.GetPos().x)
    array_speed.append(mcrank.GetAngularVelocity().z)

    
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    
    sys.DoStepDynamics(step_size)

    
    step_number += 1

    
    if time >= tend:
        break


plt.figure(figsize=(12, 4))


plt.subplot(1, 2, 1)
plt.plot(np.array(array_angle) * 180 / np.pi, np.array(array_pos), 'bo-')
plt.xlabel('Crank Angle [deg]')
plt.ylabel('Piston Position [m]')
plt.title('Position vs. Crank Angle')
plt.grid(True)


plt.subplot(1, 2, 2)
plt.plot(np.array(array_angle) * 180 / np.pi, np.array(array_speed), 'ro-')
plt.xlabel('Crank Angle [deg]')
plt.ylabel('Crank Speed [m/s]')
plt.title('Speed vs. Crank Angle')
plt.grid(True)

plt.tight_layout()
plt.show()