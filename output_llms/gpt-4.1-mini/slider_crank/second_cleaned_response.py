import pychrono.core as chrono  
import pychrono.irrlicht as chronoirr  
import matplotlib.pyplot as plt  
import numpy as np  


sys = chrono.ChSystemNSC()


crank_center = chrono.ChVector3d(-1, 0.5, 0)  
crank_rad = 0.4  
crank_thick = 0.1  
rod_length = 1.5  


mfloor = chrono.ChBodyEasyBox(3, 1, 3, 1000, True, True)  
mfloor.SetPos(chrono.ChVector3d(0, -0.5, 0))  
mfloor.SetBodyFixed(True)  
sys.Add(mfloor)  





mcrank = chrono.ChBodyEasyCylinder(crank_rad, crank_thick, 1000, True, True)
mcrank.SetPos(crank_center + chrono.ChVector3d(0, 0, -0.1))  




try:
    mcrank.SetRot(chrono.Q_ROTATE_Y_TO_Z)  
except AttributeError:
    
    mcrank.SetRot(chrono.Q_from_AngAxis(np.pi / 2, chrono.VECT_Y))  
sys.Add(mcrank)  


mrod = chrono.ChBodyEasyBox(rod_length, 0.1, 0.1, 1000, True, True)
mrod.SetPos(crank_center + chrono.ChVector3d(crank_rad + rod_length / 2, 0, 0))  
sys.Add(mrod)  


mpiston = chrono.ChBodyEasyCylinder(0.2, 0.3, 1000, True, True)
mpiston.SetPos(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0))  

try:
    mpiston.SetRot(chrono.Q_ROTATE_Y_TO_X)
except AttributeError:
    
    
    mpiston.SetRot(chrono.Q_from_AngAxis(-np.pi / 2, chrono.VECT_Z))
sys.Add(mpiston)  


my_motor = chrono.ChLinkMotorRotationSpeed()  


my_motor.Initialize(mcrank, mfloor, chrono.ChCoordsysD(crank_center))
my_angularspeed = chrono.ChFunctionConst(chrono.CH_C_PI)  
my_motor.SetMotorFunction(my_angularspeed)  
sys.Add(my_motor)  


mjointA = chrono.ChLinkLockRevolute()  

mjointA.Initialize(mrod, mcrank, chrono.ChCoordsysD(crank_center + chrono.ChVector3d(crank_rad, 0, 0)))
sys.Add(mjointA)  


mjointB = chrono.ChLinkLockRevolute()  
mjointB.Initialize(mpiston, mrod, chrono.ChCoordsysD(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0)))
sys.Add(mjointB)  


mjointC = chrono.ChLinkLockPrismatic()  
try:
    rot_jointC = chrono.Q_ROTATE_Z_TO_X
except AttributeError:
    rot_jointC = chrono.Q_from_AngAxis(-np.pi / 2, chrono.VECT_Y)  

mjointC.Initialize(mpiston, mfloor, chrono.ChCoordsysD(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0), rot_jointC))
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

time_step = 1e-3  
max_time = 20.0  




def get_crank_angle(body):
    
    
    q = body.GetRot()
    axis = chrono.Vect3d()
    angle = q.Q_to_AngAxis(axis)
    
    if axis.z < 0:
        angle = -angle
    
    angle = angle % (2 * np.pi)
    return angle



def get_piston_pos(body):
    
    pos = body.GetPos()
    return pos.x


def get_piston_speed(body):
    
    vel = body.GetPos_dt()
    return vel.x



t = 0.0
while vis.Run() and t <= max_time:
    vis.BeginScene()  
    vis.Render()  
    vis.EndScene()  

    
    sys.DoStepDynamics(time_step)
    t += time_step

    
    array_time.append(t)
    angle = get_crank_angle(mcrank)
    array_angle.append(angle)
    pos = get_piston_pos(mpiston)
    array_pos.append(pos)
    speed = get_piston_speed(mpiston)
    array_speed.append(speed)



array_time = np.array(array_time)
array_angle = np.array(array_angle)
array_pos = np.array(array_pos)
array_speed = np.array(array_speed)


fig, axs = plt.subplots(2, 1, figsize=(10, 8), sharex=True)


axs[0].plot(array_angle, array_pos, 'b-')
axs[0].set_ylabel('Position [m]')
axs[0].grid(True)
axs[0].set_title('Piston Position vs Crank Angle')


axs[1].plot(array_angle, array_speed, 'r-')
axs[1].set_xlabel('Crank Angle [rad]')
axs[1].set_ylabel('Speed [m/s]')
axs[1].grid(True)
axs[1].set_title('Piston Speed vs Crank Angle')





ticks = np.array([0, 0.5 * np.pi, np.pi, 1.5 * np.pi, 2 * np.pi])
ticklabels = [r'$0$', r'$\frac{\pi}{2}$', r'$\pi$', r'$\frac{3\pi}{2}$', r'$2\pi$']

for ax in axs:
    ax.set_xticks(ticks)
    ax.set_xticklabels(ticklabels)

plt.tight_layout()
plt.show()