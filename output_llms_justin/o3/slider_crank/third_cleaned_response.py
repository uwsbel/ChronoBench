import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import matplotlib.pyplot as plt
import numpy as np




sys = chrono.ChSystemNSC()




crank_center = chrono.ChVectorD(-1, 0.5, 0)
crank_rad    = 0.4
crank_thick  = 0.1
rod_length   = 1.5






ground = chrono.ChBodyEasyBox(3, 1, 3, 1000)
ground.SetPos(chrono.ChVectorD(0, -0.5, 0))
ground.SetFixed(True)
sys.Add(ground)


crank = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, crank_rad, crank_thick, 1000)
crank.SetPos(crank_center + chrono.ChVectorD(0, 0, -0.1))

crank.SetRot(chrono.Q_ROTATE_Y_TO_Z)
sys.Add(crank)


rod = chrono.ChBodyEasyBox(rod_length, 0.1, 0.1, 1000)
rod.SetPos(crank_center + chrono.ChVectorD(crank_rad + rod_length * 0.5, 0, 0))
sys.Add(rod)


piston = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.2, 0.3, 1000)
piston.SetPos(crank_center + chrono.ChVectorD(crank_rad + rod_length, 0, 0))
piston.SetRot(chrono.Q_ROTATE_Y_TO_X)
sys.Add(piston)






motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(crank,                     
                 ground,                    
                 chrono.ChFrameD(crank_center))  
motor_speed = chrono.ChFunction_Const(chrono.CH_PI)   
motor.SetSpeedFunction(motor_speed)
sys.Add(motor)



joint_crank_rod = chrono.ChLinkLockSpherical()
joint_crank_rod.Initialize(rod,
                           crank,
                           chrono.ChCoordsysD(crank_center + chrono.ChVectorD(crank_rad, 0, 0)))
sys.Add(joint_crank_rod)


joint_rod_piston = chrono.ChLinkLockSpherical()
joint_rod_piston.Initialize(piston,
                            rod,
                            chrono.ChCoordsysD(crank_center + chrono.ChVectorD(crank_rad + rod_length, 0, 0)))
sys.Add(joint_rod_piston)



joint_plane = chrono.ChLinkLockPlanePlane()
plane_frame = chrono.ChCoordsysD(crank_center + chrono.ChVectorD(crank_rad + rod_length, 0, 0),
                                 chrono.QUNIT)              
joint_plane.Initialize(piston, ground, plane_frame)
sys.Add(joint_plane)




vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Crank-slider – spherical & planar joints')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(1, 1, 3), chrono.ChVectorD(0, 1, 0))
vis.AddTypicalLights()




time_data   = []
angle_data  = []
pos_data    = []
speed_data  = []




step = 1e-3
while vis.Run():
    
    time_data.append(sys.GetChTime())
    angle_data.append(motor.GetMotorRot())        
    pos_data.append(piston.GetPos().x)
    speed_data.append(piston.GetPos_dt().x)

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    sys.DoStepDynamics(step)

    
    if sys.GetChTime() > 20:
        vis.GetDevice().closeDevice()




fig, (ax1, ax2) = plt.subplots(2, sharex=True)

ax1.plot(angle_data, pos_data)
ax1.set(ylabel='piston position [m]')
ax1.grid()

ax2.plot(angle_data, speed_data, 'r--')
ax2.set(ylabel='piston speed [m/s]', xlabel='crank angle [rad]')
ax2.grid()


plt.xticks(np.linspace(0, 2 * np.pi, 5),
           ['0', r'$\pi/2$', r'$\pi$', r'$3\pi/2$', r'$2\pi$'])

plt.show()