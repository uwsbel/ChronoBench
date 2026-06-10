import math
import os

import pychrono as chrono
import pychrono.irrlicht as chronoirr






try:
    VEC = chrono.ChVector3d
    FRAME = chrono.ChFramed
except AttributeError:
    VEC = chrono.ChVectorD
    FRAME = chrono.ChFrameD


def quat_axis(angle, axis):
    if hasattr(chrono, "QuatFromAngleAxis"):
        return chrono.QuatFromAngleAxis(angle, axis)
    return chrono.Q_from_AngAxis(angle, axis)


def set_gravity(system, g):
    if hasattr(system, "SetGravitationalAcceleration"):
        system.SetGravitationalAcceleration(g)
    else:
        system.Set_G_acc(g)


def make_const_function(value):
    if hasattr(chrono, "ChFunctionConst"):
        return chrono.ChFunctionConst(value)
    return chrono.ChFunction_Const(value)


def add_visual_box(body, size, local_pos=VEC(0, 0, 0), local_rot=None, color=None, texture=None):
    if local_rot is None:
        local_rot = quat_axis(0, VEC(0, 0, 1))

    shape = chrono.ChVisualShapeBox(size[0], size[1], size[2])

    if color is not None:
        try:
            shape.SetColor(color)
        except Exception:
            pass

    if texture is not None:
        try:
            shape.SetTexture(chrono.GetChronoDataFile(texture))
        except Exception:
            pass

    body.AddVisualShape(shape, FRAME(local_pos, local_rot))
    return shape


def add_visual_sphere(body, radius, local_pos=VEC(0, 0, 0), color=None):
    shape = chrono.ChVisualShapeSphere(radius)

    if color is not None:
        try:
            shape.SetColor(color)
        except Exception:
            pass

    body.AddVisualShape(shape, FRAME(local_pos, quat_axis(0, VEC(0, 0, 1))))
    return shape






if "CHRONO_DATA_DIR" in os.environ:
    chrono.SetChronoDataPath(os.environ["CHRONO_DATA_DIR"])






step_size = 1e-3

crank_radius = 0.35
rod_length = 1.20
crank_angular_speed = 2.0 * math.pi  

crank_center = VEC(0.0, 0.80, 0.0)
slider_y = crank_center.y


theta0 = math.radians(30.0)

pin_x = crank_center.x + crank_radius * math.cos(theta0)
pin_y = crank_center.y + crank_radius * math.sin(theta0)
pin = VEC(pin_x, pin_y, 0.0)


dx = math.sqrt(rod_length**2 - (pin_y - slider_y) ** 2)
wrist = VEC(pin_x + dx, slider_y, 0.0)

rod_center = VEC(
    0.5 * (pin.x + wrist.x),
    0.5 * (pin.y + wrist.y),
    0.0,
)

rod_angle = math.atan2(wrist.y - pin.y, wrist.x - pin.x)

q_identity = quat_axis(0.0, VEC(0, 0, 1))
q_crank = quat_axis(theta0, VEC(0, 0, 1))
q_rod = quat_axis(rod_angle, VEC(0, 0, 1))


q_revolute_z = q_identity



q_prismatic_x = quat_axis(math.pi / 2.0, VEC(0, 1, 0))






system = chrono.ChSystemNSC()
set_gravity(system, VEC(0, 0, 0))  

try:
    system.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_LINEARIZED)
except Exception:
    pass






floor = chrono.ChBody()
floor.SetName("fixed_truss_floor")
floor.SetFixed(True)
floor.SetPos(VEC(0, 0, 0))
system.AddBody(floor)


add_visual_box(
    floor,
    size=(4.5, 0.08, 1.0),
    local_pos=VEC(1.2, -0.04, 0),
    color=chrono.ChColor(0.55, 0.55, 0.55),
    texture="textures/concrete.jpg",
)


add_visual_box(
    floor,
    size=(0.15, 0.90, 0.25),
    local_pos=VEC(crank_center.x, crank_center.y - 0.45, -0.18),
    color=chrono.ChColor(0.35, 0.35, 0.40),
)


add_visual_box(
    floor,
    size=(2.3, 0.04, 0.04),
    local_pos=VEC(1.35, slider_y - 0.18, 0.18),
    color=chrono.ChColor(0.15, 0.15, 0.15),
)
add_visual_box(
    floor,
    size=(2.3, 0.04, 0.04),
    local_pos=VEC(1.35, slider_y + 0.18, 0.18),
    color=chrono.ChColor(0.15, 0.15, 0.15),
)






crank = chrono.ChBody()
crank.SetName("motor_driven_crankshaft")
crank.SetMass(2.0)
crank.SetInertiaXX(VEC(0.04, 0.04, 0.04))
crank.SetPos(crank_center)
crank.SetRot(q_crank)
system.AddBody(crank)


add_visual_box(
    crank,
    size=(0.16, 0.16, 0.55),
    local_pos=VEC(0, 0, 0),
    color=chrono.ChColor(0.10, 0.10, 0.10),
)


add_visual_box(
    crank,
    size=(crank_radius, 0.07, 0.08),
    local_pos=VEC(crank_radius / 2.0, 0, 0),
    color=chrono.ChColor(0.85, 0.20, 0.15),
)


add_visual_sphere(
    crank,
    radius=0.055,
    local_pos=VEC(crank_radius, 0, 0),
    color=chrono.ChColor(0.90, 0.90, 0.10),
)


add_visual_box(
    crank,
    size=(0.18, 0.12, 0.08),
    local_pos=VEC(-0.13, 0, 0),
    color=chrono.ChColor(0.55, 0.05, 0.05),
)






rod = chrono.ChBody()
rod.SetName("connecting_rod")
rod.SetMass(1.0)
rod.SetInertiaXX(VEC(0.005, 0.12, 0.12))
rod.SetPos(rod_center)
rod.SetRot(q_rod)
system.AddBody(rod)

add_visual_box(
    rod,
    size=(rod_length, 0.06, 0.06),
    local_pos=VEC(0, 0, 0),
    color=chrono.ChColor(0.10, 0.35, 0.90),
)

add_visual_sphere(
    rod,
    radius=0.07,
    local_pos=VEC(-rod_length / 2.0, 0, 0),
    color=chrono.ChColor(0.10, 0.35, 0.90),
)

add_visual_sphere(
    rod,
    radius=0.07,
    local_pos=VEC(rod_length / 2.0, 0, 0),
    color=chrono.ChColor(0.10, 0.35, 0.90),
)






piston = chrono.ChBody()
piston.SetName("sliding_piston")
piston.SetMass(1.5)
piston.SetInertiaXX(VEC(0.02, 0.02, 0.02))
piston.SetPos(wrist)
piston.SetRot(q_identity)
system.AddBody(piston)


add_visual_box(
    piston,
    size=(0.35, 0.28, 0.28),
    local_pos=VEC(0.18, 0, 0),
    color=chrono.ChColor(0.20, 0.75, 0.25),
)

add_visual_sphere(
    piston,
    radius=0.06,
    local_pos=VEC(0, 0, 0),
    color=chrono.ChColor(0.95, 0.85, 0.10),
)







motor = chrono.ChLinkMotorRotationSpeed()
motor.SetName("constant_speed_crank_motor")
motor.Initialize(crank, floor, FRAME(crank_center, q_revolute_z))
motor.SetSpeedFunction(make_const_function(crank_angular_speed))
system.AddLink(motor)


rev_crank_rod = chrono.ChLinkLockRevolute()
rev_crank_rod.SetName("crank_pin_revolute")
rev_crank_rod.Initialize(rod, crank, FRAME(pin, q_revolute_z))
system.AddLink(rev_crank_rod)


rev_rod_piston = chrono.ChLinkLockRevolute()
rev_rod_piston.SetName("wrist_pin_revolute")
rev_rod_piston.Initialize(rod, piston, FRAME(wrist, q_revolute_z))
system.AddLink(rev_rod_piston)


slider_joint = chrono.ChLinkLockPrismatic()
slider_joint.SetName("piston_slider_prismatic")
slider_joint.Initialize(piston, floor, FRAME(wrist, q_prismatic_x))
system.AddLink(slider_joint)






vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("PyChrono Crank-Slider Mechanism")
vis.Initialize()


try:
    vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
except Exception:
    try:
        vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    except Exception:
        pass

try:
    vis.AddSkyBox()
except Exception:
    pass


camera_position = VEC(1.25, 2.1, 3.2)
camera_target = VEC(0.9, 0.75, 0.0)
vis.AddCamera(camera_position, camera_target)

vis.AddTypicalLights()

try:
    vis.AddLightWithShadow(
        VEC(2.5, 5.0, 2.5),  
        VEC(0.8, 0.6, 0.0),  
        8.0,                 
        1.0,                 
        10.0,                
        60.0                 
    )
except Exception:
    pass

try:
    vis.EnableShadows()
except Exception:
    pass






realtime_timer = chrono.ChRealtimeStepTimer()

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    system.DoStepDynamics(step_size)
    realtime_timer.Spin(step_size)