import math
import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.robot as robot






def ChVec(x, y, z):
    if hasattr(chrono, "ChVector3d"):
        return chrono.ChVector3d(x, y, z)
    return chrono.ChVectorD(x, y, z)


def ChCoords(pos, rot):
    if hasattr(chrono, "ChCoordsysd"):
        return chrono.ChCoordsysd(pos, rot)
    return chrono.ChCoordsysD(pos, rot)


def QuatFromYaw(yaw):
    if hasattr(chrono, "QuatFromAngleZ"):
        return chrono.QuatFromAngleZ(yaw)
    if hasattr(chrono, "Q_from_AngZ"):
        return chrono.Q_from_AngZ(yaw)

    q = chrono.ChQuaternionD()
    q.SetFromAngleZ(yaw)
    return q


def SetBodyFixed(body, fixed=True):
    if hasattr(body, "SetFixed"):
        body.SetFixed(fixed)
    else:
        body.SetBodyFixed(fixed)


def SetGravity(system, g):
    if hasattr(system, "SetGravitationalAcceleration"):
        system.SetGravitationalAcceleration(g)
    else:
        system.Set_G_acc(g)


def MakeNSCMaterial():
    if hasattr(chrono, "ChContactMaterialNSC"):
        mat = chrono.ChContactMaterialNSC()
    else:
        mat = chrono.ChMaterialSurfaceNSC()

    if hasattr(mat, "SetFriction"):
        mat.SetFriction(0.8)
    if hasattr(mat, "SetRestitution"):
        mat.SetRestitution(0.05)

    return mat


def SetTurtlebotMotorSpeeds(bot, left_speed, right_speed):
    

    
    for name in [
        "SetMotorSpeed",
        "SetMotorSpeeds",
        "SetDriveMotorSpeeds",
        "SetWheelSpeeds",
    ]:
        if hasattr(bot, name):
            fn = getattr(bot, name)
            try:
                fn(left_speed, right_speed)
                return
            except TypeError:
                pass

    
    left_names = ["SetLeftMotorSpeed", "SetLeftWheelSpeed"]
    right_names = ["SetRightMotorSpeed", "SetRightWheelSpeed"]

    left_fn = None
    right_fn = None

    for n in left_names:
        if hasattr(bot, n):
            left_fn = getattr(bot, n)

    for n in right_names:
        if hasattr(bot, n):
            right_fn = getattr(bot, n)

    if left_fn and right_fn:
        left_fn(left_speed)
        right_fn(right_speed)
        return

    raise RuntimeError(
        "Could not find a supported Turtlebot motor-speed API in this PyChrono build."
    )


def UpdateTurtlebot(bot, time):
    for name in ["Update", "Synchronize"]:
        if hasattr(bot, name):
            fn = getattr(bot, name)
            try:
                fn(time)
            except TypeError:
                fn()
            return






step_size = 1.0e-3
render_step = 1.0 / 60.0
end_time = 12.0

terrain_size_x = 12.0
terrain_size_y = 12.0
terrain_thickness = 0.10


init_pos = ChVec(0.0, 0.0, 0.18)
init_yaw = 0.0
init_rot = QuatFromYaw(init_yaw)


forward_speed = 6.0
turn_speed = 4.0






system = chrono.ChSystemNSC()
SetGravity(system, ChVec(0.0, 0.0, -9.81))

contact_mat = MakeNSCMaterial()






try:
    ground = chrono.ChBodyEasyBox(
        terrain_size_x,
        terrain_size_y,
        terrain_thickness,
        1000.0,
        True,
        True,
        contact_mat,
    )
except TypeError:
    ground = chrono.ChBodyEasyBox(
        terrain_size_x,
        terrain_size_y,
        terrain_thickness,
        1000.0,
        True,
        True,
    )

ground.SetName("rigid terrain")
ground.SetPos(ChVec(0.0, 0.0, -terrain_thickness / 2.0))
SetBodyFixed(ground, True)
system.Add(ground)







TurtlebotClass = None
for class_name in ["TurtleBot", "Turtlebot"]:
    if hasattr(robot, class_name):
        TurtlebotClass = getattr(robot, class_name)
        break

if TurtlebotClass is None:
    raise RuntimeError("Could not find TurtleBot/Turtlebot class in pychrono.robot.")

try:
    turtlebot = TurtlebotClass(system)
except TypeError:
    turtlebot = TurtlebotClass()

init_pose = ChCoords(init_pos, init_rot)


try:
    turtlebot.Initialize(init_pose)
except TypeError:
    
    turtlebot.Initialize(system, init_pose)






vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("PyChrono Turtlebot on Rigid Terrain")
vis.Initialize()

if hasattr(vis, "AddLogo"):
    try:
        vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
    except Exception:
        pass

vis.AddSkyBox()
vis.AddCamera(ChVec(2.5, -4.0, 2.0), ChVec(0.0, 0.0, 0.25))
vis.AddTypicalLights()






def MotorSchedule(time):
    

    if time < 1.0:
        return 0.0, 0.0

    elif time < 3.0:
        return forward_speed, forward_speed

    elif time < 5.0:
        return -turn_speed, turn_speed

    elif time < 7.0:
        return forward_speed, forward_speed

    elif time < 9.0:
        return turn_speed, -turn_speed

    elif time < 11.0:
        return forward_speed, forward_speed

    else:
        return 0.0, 0.0






realtime_timer = chrono.ChRealtimeStepTimer()
render_timer = 0.0

while vis.Run():
    time = system.GetChTime()

    if time > end_time:
        break

    
    left_w, right_w = MotorSchedule(time)
    SetTurtlebotMotorSpeeds(turtlebot, left_w, right_w)

    
    UpdateTurtlebot(turtlebot, time)

    
    render_timer += step_size
    if render_timer >= render_step:
        render_timer = 0.0
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    
    system.DoStepDynamics(step_size)

    
    realtime_timer.Spin(step_size)