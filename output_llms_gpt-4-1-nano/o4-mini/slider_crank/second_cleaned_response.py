import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import matplotlib.pyplot as plt
import numpy as np




sys = chrono.ChSystemNSC()


crank_center = chrono.ChVectorD(-1, 0.5, 0)   
crank_rad    = 0.4
crank_thick  = 0.1
rod_length   = 1.5


mfloor = chrono.ChBodyEasyBox(3, 1, 3,            
                              1000,               
                              True, True)         
mfloor.SetPos(chrono.ChVectorD(0, -0.5, 0))
mfloor.SetBodyFixed(True)
sys.Add(mfloor)



mcrank = chrono.ChBodyEasyCylinder(crank_rad,
                                   crank_thick,
                                   1000,
                                   True, True)

mcrank.SetPos(crank_center)


sys.Add(mcrank)


mrod = chrono.ChBodyEasyBox(rod_length, 0.1, 0.1,
                            1000, True, True)

mrod.SetPos(crank_center + chrono.ChVectorD(crank_rad + rod_length/2, 0, 0))
sys.Add(mrod)


mpiston = chrono.ChBodyEasyCylinder(0.2,
                                    0.3,
                                    1000,
                                    True, True)
mpiston.SetPos(crank_center + chrono.ChVectorD(crank_rad + rod_length, 0, 0))
sys.Add(mpiston)





motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(mcrank,            
                 mfloor,            
                 chrono.ChFrameD(crank_center))   

motor.SetSpeedFunction(chrono.ChFunction_Const(chrono.CH_C_PI))
sys.Add(motor)


jointA = chrono.ChLinkLockRevolute()
jointA.Initialize(mcrank,
                  mrod,
                  chrono.ChFrameD(crank_center + chrono.ChVectorD(crank_rad, 0, 0)))
sys.Add(jointA)


jointB = chrono.ChLinkLockRevolute()
jointB.Initialize(mrod,
                  mpiston,
                  chrono.ChFrameD(crank_center + chrono.ChVectorD(crank_rad + rod_length, 0, 0)))
sys.Add(jointB)




q_z2x = chrono.Q_from_AngAxis(chrono.CH_C_PI/2, chrono.ChVectorD(0, 1, 0))
jointC = chrono.ChLinkLockPrismatic()
jointC.Initialize(mfloor,
                  mpiston,
                  chrono.ChFrameD(crank_center + chrono.ChVectorD(crank_rad + rod_length, 0, 0),
                                  q_z2x))
sys.Add(jointC)




vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Crank–Slider Demo')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(1, 1, 3),
              chrono.ChVectorD(0, 1, 0))
vis.AddTypicalLights()




array_time  = []
array_angle = []
array_pos   = []
array_speed = []

t_end = 20.0   
dT    = 1e-3   




while vis.Run():
    t = sys.GetChTime()
    if t > t_end:
        break

    
    array_time.append(t)
    
    angle = motor.GetMotorRot()    
    array_angle.append(angle)
    
    pos   = mpiston.GetPos().x
    speed = mpiston.GetPos_dt().x
    array_pos.append(pos)
    array_speed.append(speed)

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(dT)


vis.GetIrrlichtDevice().closeDevice()




array_angle = np.array(array_angle)
array_pos   = np.array(array_pos)
array_speed = np.array(array_speed)

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6), tight_layout=True)


ax1.plot(array_angle, array_pos, 'b-')
ax1.set_xlabel('Crank angle [rad]')
ax1.set_ylabel('Piston pos [m]')
ax1.grid(True)


ax2.plot(array_angle, array_speed, 'r-')
ax2.set_xlabel('Crank angle [rad]')
ax2.set_ylabel('Piston speed [m/s]')
ax2.grid(True)



ticks = np.arange(0, 2 * np.pi + 1e-6, np.pi/2)
labels = ['0', r'$\frac{\pi}{2}$', r'$\pi$', r'$\frac{3\pi}{2}$', r'$2\pi$']
ax1.set_xticks(ticks)
ax1.set_xticklabels(labels)
ax2.set_xticks(ticks)
ax2.set_xticklabels(labels)

plt.show()