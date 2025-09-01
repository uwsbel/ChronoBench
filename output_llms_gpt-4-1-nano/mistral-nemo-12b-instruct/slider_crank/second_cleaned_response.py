import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import matplotlib.pyplot as plt
import numpy as np


sys = chrono.ChSystemNSC()


crank_center = chrono.ChVector3d(-1, 0.5, 0)
crank_rad = 0.4
crank_thick = 0.1
rod_length = 1.5


array_time = []
array_angle = []
array_pos = []
array_speed = []




start_time = chrono.GetChronoTime()
while chrono.GetChronoTime() - start_time < 20:  
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(1e-3)

    
    array_time.append(chrono.GetChronoTime() - start_time)
    array_angle.append(mcrank.GetRot().GetYaw())  
    array_pos.append(mpiston.GetPos().x)  
    array_speed.append((array_pos[-1] - array_pos[-2]) / (1e-3))  




fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(8, 10))

ax1.plot(array_angle, array_pos, 'b-')
ax1.set_ylabel('Position [m]')
ax1.grid(True)

ax2.plot(array_angle, array_speed, 'r-')
ax2.set_xlabel('Crank angle [rad]')
ax2.set_ylabel('Speed [m/s]')
ax2.grid(True)


ax1.set_xticks(np.linspace(0, 2 * np.pi, 5))
ax2.set_xticks(np.linspace(0, 2 * np.pi, 5))

plt.show()