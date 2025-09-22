###############################################################################
# Simple pendulum example with PyChrono + Irrlicht
###############################################################################
# Prerequisites:
#   pip install pychrono
#   (the Irrlicht binaries that come with the PyChrono Windows/conda packages
#    are found automatically; on Linux make sure LD_LIBRARY_PATH is set)
###############################################################################

import math
import pychrono as chrono
import pychrono.irrlicht as chronoirr

# ---------------------------------------------------------------------------
# 1. Chrono system initialisation
# ---------------------------------------------------------------------------
chrono.SetChronoDataPath(chrono.GetChronoDataPath())           # texture / data
system = chrono.ChSystemNSC()                                  # non-smooth contact
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))               # gravity

# ---------------------------------------------------------------------------
# 2. Helper parameters
# ---------------------------------------------------------------------------
time_step      = 1.0/500.0                                    # integrator step
sim_end        = 10.0                                         # total simulation time [s]
log_interval   = 0.1                                          # how often to print [s]
next_log_time  = 0.0

# Pendulum geometry ----------------------------------------------------------
rod_length = 1.0                                              # [m]
rod_radius = 0.03                                             # [m]
rod_density= 1000                                             # [kg/m^3]
rod_volume = math.pi*rod_radius**2 * rod_length
rod_mass   = rod_density * rod_volume
rod_inertia= chrono.ChVectorD(0.5*rod_mass*rod_radius**2,
                              0.5*rod_mass*rod_radius**2,
                              (1/2)*rod_mass*(rod_radius**2)) # cylinder about center

# ---------------------------------------------------------------------------
# 3. Ground body (fixed)
# ---------------------------------------------------------------------------
ground = chrono.ChBody()
ground.SetPos(chrono.ChVectorD(0, 0, 0))
ground.SetBodyFixed(True)

# Add minimal visual asset so it shows up
ground_shape = chrono.ChBoxShape()
ground_shape.GetBoxGeometry().Size = chrono.ChVectorD(0.2, 0.02, 0.2)
ground.AddVisualShape(ground_shape)

# Must add each body to system
system.Add(ground)

# ---------------------------------------------------------------------------
# 4. Pendulum body
# ---------------------------------------------------------------------------
pendulum = chrono.ChBody()
pendulum.SetPos(chrono.ChVectorD(0, -rod_length/2.0, 0))      # CoM initially vertical

# Mass & inertia
pendulum.SetMass(rod_mass)
pendulum.SetInertiaXX(rod_inertia)

# Collision shape (disable if not needed)
collision = chrono.ChCylinderShape()
collision.GetCylinderGeometry().p1 = chrono.ChVectorD(0,  rod_length/2.0, 0)
collision.GetCylinderGeometry().p2 = chrono.ChVectorD(0, -rod_length/2.0, 0)
collision.GetCylinderGeometry().rad= rod_radius
pendulum.AddVisualShape(collision)                            # also use as visual

# Rotate so that its long axis is Y, not default X
rot = chrono.Q_from_AngAxis(math.pi/2, chrono.ChVectorD(0,0,1))
pendulum.SetRot(rot)

system.Add(pendulum)

# ---------------------------------------------------------------------------
# 5. Revolute joint between ground and pendulum
# ---------------------------------------------------------------------------
# Revolute axis is Z pointing out of the screen (Irrlicht default)
joint = chrono.ChLinkRevolute()
joint.Initialize(ground,                                   # body1
                 pendulum,                                 # body2
                 chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), # joint frame in abs coords
                                   chrono.Q_from_AngAxis(math.pi/2,
                                                         chrono.ChVectorD(1,0,0)))) # align axis
system.AddLink(joint)

# ---------------------------------------------------------------------------
# 6. Irrlicht application
# ---------------------------------------------------------------------------
app = chronoirr.ChIrrApp(system, "Simple pendulum", chronoirr.dimension2du(1024,768))
app.AddTypicalSky()
app.AddTypicalLogo()
app.AddTypicalLights()
app.AddTypicalCamera(chronoirr.vector3df(1.5,0.0,1.5), chronoirr.vector3df(0,0,0))

app.AssetBindAll()
app.AssetUpdateAll()

# ---------------------------------------------------------------------------
# 7. Simulation loop
# ---------------------------------------------------------------------------
while app.GetDevice().run():
    app.BeginScene()
    app.DrawAll()

    # Integrate dynamics
    system.DoStepDynamics(time_step)

    # Periodic logging -------------------------------------------------------
    t = system.GetChTime()
    if t >= next_log_time - 1e-9:
        tip_pos = pendulum.GetPos() + pendulum.GetRot().Rotate(chrono.ChVectorD(0, -rod_length/2.0, 0))
        tip_vel = pendulum.GetPos_dt() + pendulum.GetWvel_loc().Cross(chrono.ChVectorD(0, -rod_length/2.0, 0))
        print(f"[{t:6.3f} s]  tip pos = ({tip_pos.x:6.3f}, {tip_pos.y:6.3f}, {tip_pos.z:6.3f})  "
              f" tip vel = ({tip_vel.x:6.3f}, {tip_vel.y:6.3f}, {tip_vel.z:6.3f})")
        next_log_time += log_interval

    app.EndScene()

    # Optional stop condition without closing window automatically
    if t > sim_end:
        break