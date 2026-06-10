import math
import pychrono as chrono
import pychrono.irrlicht as chronoirr





Vec = chrono.ChVector3d if hasattr(chrono, "ChVector3d") else chrono.ChVectorD
Frame = chrono.ChFrameD if hasattr(chrono, "ChFrameD") else chrono.ChFrame
QUNIT = chrono.QUNIT if hasattr(chrono, "QUNIT") else chrono.ChQuaternionD(1, 0, 0, 0)


def set_fixed(body, fixed=True):
    if hasattr(body, "SetFixed"):
        body.SetFixed(fixed)
    else:
        body.SetBodyFixed(fixed)


def set_gravity(system, g):
    if hasattr(system, "SetGravitationalAcceleration"):
        system.SetGravitationalAcceleration(g)
    else:
        system.Set_G_acc(g)


def make_const_function(value):
    if hasattr(chrono, "ChFunctionConst"):
        return chrono.ChFunctionConst(value)
    return chrono.ChFunction_Const(value)


def make_visual_material(r, g, b, spec=0.25):
    mat = chrono.ChVisualMaterial()
    mat.SetDiffuseColor(chrono.ChColor(r, g, b))
    mat.SetAmbientColor(chrono.ChColor(0.25 * r, 0.25 * g, 0.25 * b))
    mat.SetSpecularColor(chrono.ChColor(spec, spec, spec))
    return mat


def apply_material(shape, mat):
    
    if hasattr(shape, "SetMaterial"):
        try:
            shape.SetMaterial(0, mat)
            return
        except Exception:
            try:
                shape.SetMaterial(mat)
                return
            except Exception:
                pass
    if hasattr(shape, "AddMaterial"):
        shape.AddMaterial(mat)


def set_optional(obj, names, *args):
    for name in names:
        if hasattr(obj, name):
            getattr(obj, name)(*args)
            return True
    return False





system = chrono.ChSystemNSC()
set_gravity(system, Vec(0, 0, 0))  


step_size = 1e-3
carrier_speed = 1.0  


sun_radius = 0.60
planet_radius = 0.35
gear_thickness = 0.18
carrier_radius = sun_radius + planet_radius  


mat_truss = make_visual_material(0.30, 0.32, 0.36)
mat_bar = make_visual_material(0.10, 0.45, 0.85)
mat_sun = make_visual_material(0.95, 0.55, 0.10)
mat_planet = make_visual_material(0.10, 0.75, 0.30)
mat_pin = make_visual_material(0.70, 0.70, 0.72)





def add_box_visual(body, sx, sy, sz, local_pos, local_rot, mat):
    shape = chrono.ChVisualShapeBox(sx, sy, sz)
    apply_material(shape, mat)
    body.AddVisualShape(shape, Frame(local_pos, local_rot))
    return shape


def add_cylinder_visual(body, radius, length, local_pos, local_rot, mat):
    
    shape = chrono.ChVisualShapeCylinder(radius, length)
    apply_material(shape, mat)
    body.AddVisualShape(shape, Frame(local_pos, local_rot))
    return shape


def add_gear_teeth(body, pitch_radius, tooth_len, tooth_width, thickness, n_teeth, mat):
    
    for i in range(n_teeth):
        a = 2.0 * math.pi * i / n_teeth
        q = chrono.Q_from_AngAxis(a, Vec(0, 0, 1))
        r = pitch_radius + 0.5 * tooth_len
        pos = Vec(r * math.cos(a), r * math.sin(a), 0)
        add_box_visual(body, tooth_len, tooth_width, thickness, pos, q, mat)


def create_gear(name, radius, thickness, mass, pos, mat, n_teeth):
    body = chrono.ChBody()
    body.SetName(name)
    body.SetMass(mass)

    
    Ixx = (1.0 / 12.0) * mass * (3.0 * radius * radius + thickness * thickness)
    Iyy = Ixx
    Izz = 0.5 * mass * radius * radius
    body.SetInertiaXX(Vec(Ixx, Iyy, Izz))

    body.SetPos(pos)

    add_cylinder_visual(body, radius, thickness, Vec(0, 0, 0), QUNIT, mat)
    add_gear_teeth(
        body,
        pitch_radius=radius,
        tooth_len=0.055,
        tooth_width=2.0 * math.pi * radius / n_teeth * 0.55,
        thickness=thickness * 1.05,
        n_teeth=n_teeth,
        mat=mat,
    )

    system.Add(body)
    return body





truss = chrono.ChBody()
truss.SetName("fixed_truss")
set_fixed(truss, True)
truss.SetPos(Vec(0, 0, 0))


add_box_visual(truss, 2.8, 2.2, 0.06, Vec(0.35, 0, -0.18), QUNIT, mat_truss)


add_cylinder_visual(truss, 0.09, 0.35, Vec(0, 0, 0), QUNIT, mat_pin)

system.Add(truss)





carrier = chrono.ChBody()
carrier.SetName("motor_driven_carrier_bar")
carrier_mass = 1.0
carrier.SetMass(carrier_mass)

bar_len = carrier_radius
bar_w = 0.12
bar_t = 0.10
Ixx = (1.0 / 12.0) * carrier_mass * (bar_w * bar_w + bar_t * bar_t)
Iyy = (1.0 / 12.0) * carrier_mass * (bar_len * bar_len + bar_t * bar_t)
Izz = (1.0 / 12.0) * carrier_mass * (bar_len * bar_len + bar_w * bar_w)
carrier.SetInertiaXX(Vec(Ixx, Iyy, Izz))


carrier.SetPos(Vec(carrier_radius / 2.0, 0, 0))

add_box_visual(carrier, bar_len, bar_w, bar_t, Vec(0, 0, 0), QUNIT, mat_bar)
add_cylinder_visual(carrier, 0.11, 0.22, Vec(-bar_len / 2.0, 0, 0), QUNIT, mat_pin)
add_cylinder_visual(carrier, 0.08, 0.22, Vec(+bar_len / 2.0, 0, 0), QUNIT, mat_pin)

system.Add(carrier)






sun = create_gear(
    name="fixed_sun_gear",
    radius=sun_radius,
    thickness=gear_thickness,
    mass=2.0,
    pos=Vec(0, 0, 0),
    mat=mat_sun,
    n_teeth=36,
)
set_fixed(sun, True)


planet = create_gear(
    name="planet_gear",
    radius=planet_radius,
    thickness=gear_thickness,
    mass=1.0,
    pos=Vec(carrier_radius, 0, 0),
    mat=mat_planet,
    n_teeth=21,
)





carrier_motor = chrono.ChLinkMotorRotationSpeed()
carrier_motor.SetName("carrier_constant_speed_motor")


carrier_motor.Initialize(carrier, truss, Frame(Vec(0, 0, 0), QUNIT))
carrier_motor.SetSpeedFunction(make_const_function(carrier_speed))

system.AddLink(carrier_motor)





planet_revolute = chrono.ChLinkLockRevolute()
planet_revolute.SetName("planet_revolute_joint")


planet_revolute.Initialize(planet, carrier, Frame(Vec(carrier_radius, 0, 0), QUNIT))

system.AddLink(planet_revolute)





gear_link = chrono.ChLinkLockGear()
gear_link.SetName("sun_planet_epicyclic_gear_constraint")


gear_link.Initialize(sun, planet, Frame(Vec(0, 0, 0), QUNIT))



set_optional(gear_link, ["SetFrameShaft1", "Set_shaft1"], Frame(Vec(0, 0, 0), QUNIT))
set_optional(gear_link, ["SetFrameShaft2", "Set_shaft2"], Frame(Vec(0, 0, 0), QUNIT))



set_optional(
    gear_link,
    ["SetTransmissionRatio", "Set_transmission_ratio", "SetTau", "Set_tau"],
    -sun_radius / planet_radius,
)


set_optional(gear_link, ["SetEpicyclic", "Set_epicyclic"], True)


set_optional(gear_link, ["SetEnforcePhase", "Set_checkphase"], True)
set_optional(gear_link, ["SetPhase", "Set_phase"], 0.0)

system.AddLink(gear_link)





vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 800)
vis.SetWindowTitle("PyChrono Epicyclic Gear Demonstration")
vis.Initialize()

vis.AddSkyBox()
vis.AddCamera(Vec(0.55, -3.0, 2.0), Vec(0.35, 0.0, 0.0))
vis.AddTypicalLights()


if hasattr(vis, "EnableShadows"):
    vis.EnableShadows()





while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    system.DoStepDynamics(step_size)