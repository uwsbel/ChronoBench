"""
Crank–slider example, corrected and augmented
--------------------------------------------
This script

1. fixes a few API-related errors that were present in the original code
   (wrong class names, wrong frame object, wrong function names …),
2. stores the simulation data in four Python lists
   (time, crank angle, piston position and piston speed),
3. stops the interactive simulation after 20 s,
4. finally creates two Matplotlib sub-plots:
      – piston position  vs. crank angle
      – piston speed     vs. crank angle
   with the abscissa ticks expressed in multiples of π.
"""

import numpy as np
import matplotlib.pyplot as plt
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# --------------------------------------------------------------------------
# 1. Chrono system
# --------------------------------------------------------------------------

sys = chrono.ChSystemNSC()
sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# --------------------------------------------------------------------------
# 2. Geometric/physical parameters
# --------------------------------------------------------------------------

crank_center = chrono.ChVectorD(-1.0, 0.50, 0.0)     # crankshaft center
crank_rad    = 0.40                                  # crank radius     [m]
crank_thick  = 0.10                                  # crank thickness  [m]
rod_length   = 1.50                                  # connecting rod   [m]

rho = 1000                                           # density          [kg/m³]

# --------------------------------------------------------------------------
# 3. Bodies
# --------------------------------------------------------------------------

# ––– floor / truss ---------------------------------------------------------
mfloor = chrono.ChBodyEasyBox(3, 1, 3, rho)
mfloor.SetPos(chrono.ChVectorD(0, -0.50, 0))
mfloor.SetFixed(True)
sys.Add(mfloor)

# ––– crank (cylinder, Y-axis by default, rotate it to match Z) ------------
mcrank = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, crank_rad,
                                   crank_thick, rho)
rot_y2z = chrono.Q_from_AngAxis(-np.pi/2, chrono.ChVectorD(1, 0, 0))
mcrank.SetPos(crank_center + chrono.ChVectorD(0, 0, -0.10))
mcrank.SetRot(rot_y2z)
sys.Add(mcrank)

# ––– connecting rod (box) --------------------------------------------------
mrod = chrono.ChBodyEasyBox(rod_length, 0.10, 0.10, rho)
mrod.SetPos(crank_center +
            chrono.ChVectorD(crank_rad + rod_length / 2.0, 0, 0))
sys.Add(mrod)

# ––– piston (cylinder, rotate Y → X i.e. +90° around Z) --------------------
mpiston = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.20, 0.30, rho)
rot_y2x = chrono.Q_from_AngAxis(+np.pi/2, chrono.ChVectorD(0, 0, 1))
mpiston.SetPos(crank_center +
               chrono.ChVectorD(crank_rad + rod_length, 0, 0))
mpiston.SetRot(rot_y2x)
sys.Add(mpiston)

# --------------------------------------------------------------------------
# 4. Joints / motor
# --------------------------------------------------------------------------

# ––– crank driven by speed motor ------------------------------------------
motor = chrono.ChLinkMotorRotationSpeed()
frame_crank = chrono.ChFrameD(crank_center)          # motor location
motor.Initialize(mcrank, mfloor, frame_crank)

speed_fun  = chrono.ChFunction_Const(np.pi)          # ω = π rad/s
motor.SetSpeedFunction(speed_fun)
sys.AddLink(motor)

# ––– revolute crank–rod ----------------------------------------------------
rev_A = chrono.ChLinkLockRevolute()
pos_A = chrono.ChVectorD(crank_center.x + crank_rad, crank_center.y, 0)
rev_A.Initialize(mrod, mcrank, chrono.ChFrameD(pos_A))
sys.AddLink(rev_A)

# ––– revolute rod–piston ---------------------------------------------------
rev_B = chrono.ChLinkLockRevolute()
pos_B = chrono.ChVectorD(crank_center.x + crank_rad + rod_length,
                         crank_center.y, 0)
rev_B.Initialize(mpiston, mrod, chrono.ChFrameD(pos_B))
sys.AddLink(rev_B)

# ––– prismatic piston–floor (translation along X) -------------------------
pris_C = chrono.ChLinkLockPrismatic()
q_z2x = chrono.Q_from_AngAxis(-np.pi/2, chrono.ChVectorD(0, 1, 0))  # Z→X
pris_C.Initialize(mpiston, mfloor, chrono.ChFrameD(pos_B, q_z2x))
sys.AddLink(pris_C)

# --------------------------------------------------------------------------
# 5. Irrlicht visualisation
# --------------------------------------------------------------------------

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Crank–slider demo')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(1, 1, 3), chrono.ChVectorD(0, 1, 0))
vis.AddTypicalLights()

# --------------------------------------------------------------------------
# 6. Arrays for post-processing (instruction 3)
# --------------------------------------------------------------------------

array_time  = []
array_angle = []
array_pos   = []
array_speed = []

# --------------------------------------------------------------------------
# 7. Simulation loop (instructions 4 & 5)
# --------------------------------------------------------------------------

step = 1e-3                       # 1 ms
time_end = 20.0                   # stop after 20 s

while vis.Run():
    current_time = sys.GetChTime()
    if current_time > time_end:
        break

    # ---------- data collection -------------------------------------------
    omega = speed_fun.Get_y(current_time)            # motor angular speed
    crank_angle = omega * current_time               # since ω is constant
    piston_pos   = mpiston.GetPos().x
    piston_speed = mpiston.GetPos_dt().x

    array_time.append(current_time)
    array_angle.append(crank_angle)
    array_pos.append(piston_pos)
    array_speed.append(piston_speed)
    # ----------------------------------------------------------------------

    # draw & advance --------------------------------------------------------
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    sys.DoStepDynamics(step)

# --------------------------------------------------------------------------
# 8. Plotting (instruction 6)
# --------------------------------------------------------------------------

array_angle = np.array(array_angle)
array_pos   = np.array(array_pos)
array_speed = np.array(array_speed)

# build π-based ticks (0, π/2, π, 3π/2, …) up to the last stored angle
xtick_max = np.ceil(array_angle.max() / (np.pi/2)) * (np.pi/2)
xticks = np.arange(0, xtick_max + 1e-9, np.pi/2)
xtick_labels = []
for x in xticks:
    if np.isclose(x, 0.0):
        xtick_labels.append('0')
    else:
        mult = x / np.pi
        if np.isclose(mult % 1, 0):          # integer multiple (π, 2π, …)
            xtick_labels.append(f'{int(mult)}π')
        else:                                # half-integer multiple (π/2,…)
            xtick_labels.append(f'{int(mult*2)}/2 π')

# figure --------------------------------------------------------------------
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)

# ––– first subplot : position ---------------------------------------------
ax1.plot(array_angle, array_pos, 'b')
ax1.set_ylabel('Position  [m]')
ax1.grid(True)

# ––– second subplot : speed ------------------------------------------------
ax2.plot(array_angle, array_speed, 'r')
ax2.set_xlabel('Crank angle  [rad]')
ax2.set_ylabel('Speed  [m/s]')
ax2.grid(True)

# ––– common x-ticks --------------------------------------------------------
ax2.set_xticks(xticks)
ax2.set_xticklabels(xtick_labels)

plt.tight_layout()
plt.show()