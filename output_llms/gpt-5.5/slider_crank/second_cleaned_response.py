import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import matplotlib.pyplot as plt
import numpy as np


sys = chrono.ChSystemNSC()




crank_center = chrono.ChVector3d(-1, 0.5, 0)
crank_rad = 0.4
crank_thick = 0.1
rod_length = 1.5

time_step = 1e-3
end_time = 20.0




mfloor = chrono.ChBodyEasyBox(3, 1, 3, 1000)
mfloor.SetPos(chrono.ChVector3d(0, -0.5, 0))
mfloor.SetFixed(True)
sys.Add(mfloor)




mcrank = chrono.ChBodyEasyCylinder(
    chrono.ChAxis_Y,
    crank_rad,
    crank_thick,
    1000
)
mcrank.SetPos(crank_center + chrono.ChVector3d(0, 0, -0.1))
mcrank.SetRot(chrono.Q_ROTATE_Y_TO_Z)
sys.Add(mcrank)




mrod = chrono.ChBodyEasyBox(rod_length, 0.1, 0.1, 1000)
mrod.SetPos(crank_center + chrono.ChVector3d(crank_rad + rod_length / 2, 0, 0))
sys.Add(mrod)




mpiston = chrono.ChBodyEasyCylinder(
    chrono.ChAxis_Y,
    0.2,
    0.3,
    1000
)
mpiston.SetPos(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0))
mpiston.SetRot(chrono.Q_ROTATE_Y_TO_X)
sys.Add(mpiston)




my_motor = chrono.ChLinkMotorRotationSpeed()
my_motor.Initialize(mcrank, mfloor, chrono.ChFramed(crank_center))

my_angularspeed = chrono.ChFunctionConst(np.pi)


my_motor.SetSpeedFunction(my_angularspeed)

sys.Add(my_motor)




mjointA = chrono.ChLinkLockRevolute()
mjointA.Initialize(
    mrod,
    mcrank,
    chrono.ChFramed(crank_center + chrono.ChVector3d(crank_rad, 0, 0))
)
sys.Add(mjointA)




mjointB = chrono.ChLinkLockRevolute()
mjointB.Initialize(
    mpiston,
    mrod,
    chrono.ChFramed(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0))
)
sys.Add(mjointB)





mjointC = chrono.ChLinkLockPrismatic()
mjointC.Initialize(
    mpiston,
    mfloor,
    chrono.ChFramed(
        crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0),
        chrono.Q_ROTATE_Z_TO_X
    )
)
sys.Add(mjointC)




array_time = []
array_angle = []
array_pos = []
array_speed = []


def collect_data():
    
    array_time.append(float(sys.GetChTime()))
    array_angle.append(float(my_motor.GetMotorRot()))
    array_pos.append(float(mpiston.GetPos().x))
    array_speed.append(float(mpiston.GetPosDt().x))



collect_data()




vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Crank demo")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(
    chrono.ChVector3d(1, 1, 3),
    chrono.ChVector3d(0, 1, 0)
)
vis.AddTypicalLights()




while vis.Run():
    if sys.GetChTime() >= end_time:
        break

    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    sys.DoStepDynamics(time_step)

    collect_data()




array_time = np.array(array_time)
array_angle = np.array(array_angle)
array_pos = np.array(array_pos)
array_speed = np.array(array_speed)




def pi_tick_label(k):
    
    if k == 0:
        return "0"

    if k % 2 == 0:
        n = k // 2
        if n == 1:
            return r"$\pi$"
        return rf"${n}\pi$"

    if k == 1:
        return r"$\pi/2$"

    return rf"${k}\pi/2$"



tick_step = np.pi / 2
max_angle = np.max(array_angle)

xticks = np.arange(0, max_angle + tick_step, tick_step)
xtick_labels = [pi_tick_label(int(round(t / tick_step))) for t in xticks]




fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)


axes[0].plot(array_angle, array_pos, "b-")
axes[0].set_ylabel("Position [m]")
axes[0].set_title("Piston Position vs Crank Angle")
axes[0].grid(True)


axes[1].plot(array_angle, array_speed, "r-")
axes[1].set_xlabel("Crank angle [rad]")
axes[1].set_ylabel("Speed [m/s]")
axes[1].set_title("Piston Speed vs Crank Angle")
axes[1].grid(True)


axes[1].set_xticks(xticks)
axes[1].set_xticklabels(xtick_labels)

plt.tight_layout()
plt.show()