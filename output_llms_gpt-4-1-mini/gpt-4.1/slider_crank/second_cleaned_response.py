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


mcrank = chrono.ChBodyEasyCylinder(chrono.VECT_Y, crank_rad, crank_thick, 1000)
mcrank.SetPos(crank_center + chrono.ChVectorD(0, 0, -0.1))

mcrank.SetRot(chrono.Q_ROTATE_X_TO_Y)
sys.Add(mcrank)


mrod = chrono.ChBodyEasyBox(rod_length, 0.1, 0.1, 1000)
mrod.SetPos(crank_center + chrono.ChVectorD(crank_rad + rod_length / 2, 0, 0))
sys.Add(mrod)


mpiston = chrono.ChBodyEasyCylinder(chrono.VECT_Y, 0.2, 0.3, 1000)
mpiston.SetPos(crank_center + chrono.ChVectorD(crank_rad + rod_length, 0, 0))
mpiston.SetRot(chrono.Q_ROTATE_Y_TO_X)
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



q_prism = chrono.Q_ROTATE_Z_TO_X
mjointC = chrono.ChLinkLockPrismatic()
mjointC.Initialize(mpiston, mfloor, chrono.ChFrameD(crank_center + chrono.ChVectorD(crank_rad + rod_length, 0, 0), q_prism))
sys.Add(mjointC)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Crank demo')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(1, 1, 3), chrono.ChVectorD(0, 0.5, 0))
vis.AddTypicalLights()


array_time = []
array_angle = []
array_pos = []
array_speed = []


t_end = 20.0  
timestep = 1e-3


while vis.Run():
    t = sys.GetChTime()
    
    
    crank_rot = mcrank.GetRot()
    
    
    
    crank_angle = my_angularspeed.Get_y(t) * t  

    
    piston_pos = mpiston.GetPos().x
    
    piston_speed = mpiston.GetPos_dt().x

    array_time.append(t)
    array_angle.append(crank_angle)
    array_pos.append(piston_pos)
    array_speed.append(piston_speed)

    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(timestep)

    
    if t >= t_end:
        break


array_angle = np.array(array_angle)
array_pos = np.array(array_pos)
array_speed = np.array(array_speed)

fig, axs = plt.subplots(2, 1, figsize=(10, 8), sharex=True)


axs[0].plot(array_angle, array_pos, color='b')
axs[0].set_ylabel('Piston Position [m]')
axs[0].set_title('Piston Position vs Crank Angle')


axs[1].plot(array_angle, array_speed, color='r')
axs[1].set_xlabel('Crank Angle [rad]')
axs[1].set_ylabel('Piston Speed [m/s]')
axs[1].set_title('Piston Speed vs Crank Angle')


max_angle = np.max(array_angle)
xticks = np.arange(0, max_angle + 0.1, np.pi / 2)
xtick_labels = [r'$0$'] + [rf'${n}\pi/2$' if n % 2 else rf'${n//2}\pi$' for n in range(1, int(2*max_angle/np.pi)+2)]
axs[1].set_xticks(xticks)
axs[1].set_xticklabels([rf'${{:.0f}}\pi$'.format(x/np.pi) if x%np.pi==0 and x!=0 else (rf'${{:.0f}}\pi/2$'.format(x/(np.pi/2)) if x!=0 else r'$0$') for x in xticks])

plt.tight_layout()
plt.show()