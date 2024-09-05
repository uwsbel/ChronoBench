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


mcrank = chrono.ChBodyEasyCylinder(crank_rad, crank_thick, 1000, chrono.ChAxis_Y)
mcrank.SetPos(crank_center + chrono.ChVectorD(0, 0, -0.1))

mcrank.SetRot(chrono.Q_ROTATE_Y_TO_Z)
sys.Add(mcrank)


mrod = chrono.ChBodyEasyBox(rod_length, 0.1, 0.1, 1000)
mrod.SetPos(crank_center + chrono.ChVectorD(crank_rad + rod_length / 2, 0, 0))
sys.Add(mrod)


mpiston = chrono.ChBodyEasyCylinder(0.2, 0.3, 1000, chrono.ChAxis_Y)
mpiston.SetPos(crank_center + chrono.ChVectorD(crank_rad + rod_length, 0, 0))
mpiston.SetRot(chrono.Q_ROTATE_Y_TO_X)
sys.Add(mpiston)


my_motor = chrono.ChLinkMotorRotationSpeed()
my_motor.Initialize(mcrank,  
                    mfloor,  
                    chrono.ChFrameD(crank_center))  
my_angularspeed = chrono.ChFunction_Const(chrono.CH_C_PI)  
my_motor.SetSpeedFunction(my_angularspeed)
sys.Add(my_motor)


mjointA = chrono.ChLinkSpherical()
mjointA.Initialize(mrod,
                   mcrank,
                   chrono.ChCoordsysD(crank_center + chrono.ChVectorD(crank_rad, 0, 0)))
sys.Add(mjointA)


mjointB = chrono.ChLinkSpherical()
mjointB.Initialize(mpiston,
                   mrod,
                   chrono.ChCoordsysD(crank_center + chrono.ChVectorD(crank_rad + rod_length, 0, 0)))
sys.Add(mjointB)


mjointC = chrono.ChLinkLockPlanePlane()
mjointC.Initialize(mpiston,
                   mfloor,
                   chrono.ChCoordsysD(crank_center + chrono.ChVectorD(crank_rad + rod_length, 0, 0),
                                      chrono.Q_ROTATE_Y_TO_Z)
                   )
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


while vis.Run():
    
    array_time.append(sys.GetChTime())
    array_angle.append(my_motor.GetMotorRot())
    array_pos.append(mpiston.GetPos().x)
    array_speed.append(mpiston.GetPos_dt().x)
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(1e-3)

    
    if sys.GetChTime() > 20:
        vis.GetDevice().closeDevice()


fig, (ax1, ax2) = plt.subplots(2, sharex=True)

ax1.plot(array_angle, array_pos)
ax1.set(ylabel='position [m]')
ax1.grid()

ax2.plot(array_angle, array_speed, 'r--')
ax2.set(ylabel='speed [m]', xlabel='angle [rad]')
ax2.grid()


plt.xticks(np.linspace(0, 2 * np.pi, 5), ['0', r'$\pi/2$', r'$\pi$', r'$3\pi/2$', r'$2\pi$'])
plt.show()