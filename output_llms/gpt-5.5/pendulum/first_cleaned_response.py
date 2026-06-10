import math
import pychrono as chrono
import pychrono.irrlicht as chronoirr






Vec = chrono.ChVector3d if hasattr(chrono, "ChVector3d") else chrono.ChVectorD
Coordsys = chrono.ChCoordsysD if hasattr(chrono, "ChCoordsysD") else chrono.ChCoordsysd


def set_fixed(body, fixed=True):
    if hasattr(body, "SetFixed"):
        body.SetFixed(fixed)
    else:
        body.SetBodyFixed(fixed)


def disable_collision(body):
    if hasattr(body, "EnableCollision"):
        body.EnableCollision(False)
    elif hasattr(body, "SetCollide"):
        body.SetCollide(False)


def quat_from_angle_z(angle):
    axis_z = Vec(0, 0, 1)
    if hasattr(chrono, "QuatFromAngleAxis"):
        return chrono.QuatFromAngleAxis(angle, axis_z)
    elif hasattr(chrono, "Q_from_AngAxis"):
        return chrono.Q_from_AngAxis(angle, axis_z)
    elif hasattr(chrono, "QuatFromAngleZ"):
        return chrono.QuatFromAngleZ(angle)
    else:
        return chrono.Q_from_AngZ(angle)


def get_body_velocity(body):
    if hasattr(body, "GetPosDt"):
        return body.GetPosDt()
    return body.GetPos_dt()


def vcomp(v, name):
    value = getattr(v, name)
    return value() if callable(value) else value


def vec_to_tuple(v):
    return vcomp(v, "x"), vcomp(v, "y"), vcomp(v, "z")






step_size = 1e-3
sim_time = 20.0
log_interval = 0.1

gravity = Vec(0, -9.81, 0)

pendulum_length = 2.0
pendulum_mass = 1.0
pendulum_width = 0.08
initial_angle_deg = 35.0
initial_angle = math.radians(initial_angle_deg)

pivot_pos = Vec(0, 0, 0)


com_pos = Vec(
    0.5 * pendulum_length * math.sin(initial_angle),
    -0.5 * pendulum_length * math.cos(initial_angle),
    0,
)


body_rot = quat_from_angle_z(initial_angle + math.pi)






system = chrono.ChSystemNSC()

if hasattr(system, "SetGravitationalAcceleration"):
    system.SetGravitationalAcceleration(gravity)
else:
    system.Set_G_acc(gravity)






ground = chrono.ChBodyEasyBox(
    0.35, 0.35, 0.12,     
    1000.0,              
    True,                
    False                
)

ground.SetPos(pivot_pos)
set_fixed(ground, True)
disable_collision(ground)

system.Add(ground)






density = pendulum_mass / (pendulum_width * pendulum_length * pendulum_width)

pendulum = chrono.ChBodyEasyBox(
    pendulum_width,
    pendulum_length,
    pendulum_width,
    density,
    True,
    False
)

pendulum.SetMass(pendulum_mass)


Ixx = (1.0 / 12.0) * pendulum_mass * (pendulum_length ** 2 + pendulum_width ** 2)
Iyy = (1.0 / 12.0) * pendulum_mass * (pendulum_width ** 2 + pendulum_width ** 2)
Izz = (1.0 / 12.0) * pendulum_mass * (pendulum_length ** 2 + pendulum_width ** 2)

pendulum.SetInertiaXX(Vec(Ixx, Iyy, Izz))
pendulum.SetPos(com_pos)
pendulum.SetRot(body_rot)
disable_collision(pendulum)

system.Add(pendulum)







revolute = chrono.ChLinkLockRevolute()
revolute.Initialize(
    ground,
    pendulum,
    Coordsys(pivot_pos, quat_from_angle_z(0.0))
)

system.AddLink(revolute)






vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowTitle("PyChrono Simple Pendulum")
vis.SetWindowSize(1024, 768)

if hasattr(vis, "SetCameraVertical") and hasattr(chrono, "CameraVerticalDir_Y"):
    vis.SetCameraVertical(chrono.CameraVerticalDir_Y)

vis.Initialize()
vis.AddSkyBox()
vis.AddCamera(Vec(0, 0.5, 5.0), Vec(0, -0.8, 0))
vis.AddTypicalLights()

realtime_timer = chrono.ChRealtimeStepTimer() if hasattr(chrono, "ChRealtimeStepTimer") else None






next_log_time = 0.0

print("time, pos_x, pos_y, pos_z, vel_x, vel_y, vel_z")

while vis.Run() and system.GetChTime() < sim_time:
    current_time = system.GetChTime()

    if current_time >= next_log_time:
        pos = pendulum.GetPos()
        vel = get_body_velocity(pendulum)

        px, py, pz = vec_to_tuple(pos)
        vx, vy, vz = vec_to_tuple(vel)

        print(f"{current_time:.3f}, {px:.6f}, {py:.6f}, {pz:.6f}, "
              f"{vx:.6f}, {vy:.6f}, {vz:.6f}")

        next_log_time += log_interval

    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    system.DoStepDynamics(step_size)

    if realtime_timer:
        realtime_timer.Spin(step_size)