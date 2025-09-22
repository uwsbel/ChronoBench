import numpy as np
import matplotlib.pyplot as plt
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr





sys = chrono.ChSystemNSC()
sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))





crank_center = chrono.ChVectorD(-1.0, 0.50, 0.0)     
crank_rad    = 0.40                                  
crank_thick  = 0.10                                  
rod_length   = 1.50                                  

rho = 1000                                           






mfloor = chrono.ChBodyEasyBox(3, 1, 3, rho)
mfloor.SetPos(chrono.ChVectorD(0, -0.50, 0))
mfloor.SetFixed(True)
sys.Add(mfloor)


mcrank = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, crank_rad,
                                   crank_thick, rho)
rot_y2z = chrono.Q_from_AngAxis(-np.pi/2, chrono.ChVectorD(1, 0, 0))
mcrank.SetPos(crank_center + chrono.ChVectorD(0, 0, -0.10))
mcrank.SetRot(rot_y2z)
sys.Add(mcrank)


mrod = chrono.ChBodyEasyBox(rod_length, 0.10, 0.10, rho)
mrod.SetPos(crank_center +
            chrono.ChVectorD(crank_rad + rod_length / 2.0, 0, 0))
sys.Add(mrod)


mpiston = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.20, 0.30, rho)
rot_y2x = chrono.Q_from_AngAxis(+np.pi/2, chrono.ChVectorD(0, 0, 1))
mpiston.SetPos(crank_center +
               chrono.ChVectorD(crank_rad + rod_length, 0, 0))
mpiston.SetRot(rot_y2x)
sys.Add(mpiston)






motor = chrono.ChLinkMotorRotationSpeed()
frame_crank = chrono.ChFrameD(crank_center)          
motor.Initialize(mcrank, mfloor, frame_crank)

speed_fun  = chrono.ChFunction_Const(np.pi)          
motor.SetSpeedFunction(speed_fun)
sys.AddLink(motor)


rev_A = chrono.ChLinkLockRevolute()
pos_A = chrono.ChVectorD(crank_center.x + crank_rad, crank_center.y, 0)
rev_A.Initialize(mrod, mcrank, chrono.ChFrameD(pos_A))
sys.AddLink(rev_A)


rev_B = chrono.ChLinkLockRevolute()
pos_B = chrono.ChVectorD(crank_center.x + crank_rad + rod_length,
                         crank_center.y, 0)
rev_B.Initialize(mpiston, mrod, chrono.ChFrameD(pos_B))
sys.AddLink(rev_B)


pris_C = chrono.ChLinkLockPrismatic()
q_z2x = chrono.Q_from_AngAxis(-np.pi/2, chrono.ChVectorD(0, 1, 0))  
pris_C.Initialize(mpiston, mfloor, chrono.ChFrameD(pos_B, q_z2x))
sys.AddLink(pris_C)





vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Crank–slider demo')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(1, 1, 3), chrono.ChVectorD(0, 1, 0))
vis.AddTypicalLights()





array_time  = []
array_angle = []
array_pos   = []
array_speed = []





step = 1e-3                       
time_end = 20.0                   

while vis.Run():
    current_time = sys.GetChTime()
    if current_time > time_end:
        break

    
    omega = speed_fun.Get_y(current_time)            
    crank_angle = omega * current_time               
    piston_pos   = mpiston.GetPos().x
    piston_speed = mpiston.GetPos_dt().x

    array_time.append(current_time)
    array_angle.append(crank_angle)
    array_pos.append(piston_pos)
    array_speed.append(piston_speed)
    

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    sys.DoStepDynamics(step)





array_angle = np.array(array_angle)
array_pos   = np.array(array_pos)
array_speed = np.array(array_speed)


xtick_max = np.ceil(array_angle.max() / (np.pi/2)) * (np.pi/2)
xticks = np.arange(0, xtick_max + 1e-9, np.pi/2)
xtick_labels = []
for x in xticks:
    if np.isclose(x, 0.0):
        xtick_labels.append('0')
    else:
        mult = x / np.pi
        if np.isclose(mult % 1, 0):          
            xtick_labels.append(f'{int(mult)}π')
        else:                                
            xtick_labels.append(f'{int(mult*2)}/2 π')


fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)


ax1.plot(array_angle, array_pos, 'b')
ax1.set_ylabel('Position  [m]')
ax1.grid(True)


ax2.plot(array_angle, array_speed, 'r')
ax2.set_xlabel('Crank angle  [rad]')
ax2.set_ylabel('Speed  [m/s]')
ax2.grid(True)


ax2.set_xticks(xticks)
ax2.set_xticklabels(xtick_labels)

plt.tight_layout()
plt.show()