import pychrono as chrono
import pychrono Irrlicht as chronoirr

# 1. Initialize the PyChrono environment and core components.
chrono.SetChronoDataPath(chrono.GetChronoDataPath())

# Create system
system = chrono.ChSystemNSC()

# Set gravity
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# 2. Add the required physical systems and objects as specified.

# Ground body - fixed in space
ground = chrono.ChBody()
ground.SetBodyFixed(True)
system.Add(ground)

# Mass body - a simple box
mass = chrono.ChBody()
mass.SetMass(1.0)
mass.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))
mass.SetPos(chrono.ChVectorD(0, -1, 0))
mass.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))
mass.SetBodyFixed(False)
system.Add(mass)

# 3. Create the spring-damper (ChLinkTSDA) connecting mass to ground

spring_damper = chrono.ChLinkTSDA()
spring_damper.Initialize(
    ground,                # body A (ground)
    mass,                  # body B (mass)
    False,                 # do not use frame offsets
    chrono.ChVectorD(0, 0, 0),    # pos on ground
    chrono.ChVectorD(0, 0, 0)     # pos on mass
)

# Default linear spring-damper parameters
spring_const = 50    # N/m
damping_const = 2    # Ns/m
rest_length = 1.0    # m

spring_damper.Set_springRestLength(rest_length)
spring_damper.Set_springK(spring_const)
spring_damper.Set_springR(damping_const)

system.AddLink(spring_damper)

# 4. Setup Irrlicht visualization:

application = chronoirr.ChIrrApp(system, "Mass-Spring-Damper Example", chronoirr.dimension2du(800, 600))

application.AddTypicalLogo()
application.AddTypicalSky()
application.AddTypicalLights()
application.AddTypicalCamera(chronoirr.vector3df(1.5, 1.0, 4), chronoirr.vector3df(0, -1, 0))  # position camera in front of mass

# Visual shape for ground - a large box (invisible since fixed, but good to add)
mvisual_ground = chrono.ChVisualShapeBox()
mvisual_ground.SetBoxSize(chrono.ChVectorD(2, 0.05, 2))
mvisual_ground.SetColor(chrono.ChColor(0.3, 0.3, 0.3))
ground.AddVisualShape(mvisual_ground, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))

# Visual shape for mass - a box
box_shape = chrono.ChBoxShape()
box_shape.GetBoxGeometry().Size = chrono.ChVectorD(0.2, 0.2, 0.2)
box_shape.SetColor(chrono.ChColor(0.2, 0.7, 0.2))
mass.AddVisualShape(box_shape)

# Visual shape for spring-damper (line segment between two points)
class SpringVisualizer(chrono.ChVisualShape):
    def __init__(self, link_tsda):
        super().__init__()
        self.link = link_tsda

    def Update(self):
        # Called every frame to update the visual
        pass

# Instead, we can use a simple line shape updated each frame:

line_shape = chrono.ChLineShape()
line_shape.SetColor(chrono.ChColor(0.9, 0.1, 0.1))
application.GetSceneManager().getRootSceneNode().addChild(line_shape)

def update_spring_line():
    try:
        # Get global positions of spring endpoints
        pA = spring_damper.GetMarker1().GetAbsCoord().pos
        pB = spring_damper.GetMarker2().GetAbsCoord().pos

        # Convert to irrlicht vectors
        vA = chronoirr.vector3df(float(pA.x), float(pA.y), float(pA.z))
        vB = chronoirr.vector3df(float(pB.x), float(pB.y), float(pB.z))

        # Update the line shape to draw a line from vA to vB
        # Since the built-in ChLineShape does not support direct line segment updates in pychrono python,
        # as a workaround, we can add a ChCylinderShape or create a mesh for the spring.
        # For simplicity, we will add a ChCylinderShape between mass and ground to simulate the spring

    except:
        pass

# Instead of the above, let's create a cylinder visual between the two points, updated each frame

# Create a cylinder visual shape attached to a dummy body
cyl = chrono.ChBody()
cyl.SetBodyFixed(True)  # no physics to this visual only body
system.Add(cyl)

cyl_shape = chrono.ChCylinderShape()
cyl_shape.GetCylinderGeometry().p1 = chrono.ChVectorD(0, 0, 0)
cyl_shape.GetCylinderGeometry().p2 = chrono.ChVectorD(0, 0, 1)  # initial axis (will update)
cyl_shape.GetCylinderGeometry().rad = 0.03
cyl_shape.SetColor(chrono.ChColor(0.7, 0.2, 0.2))
cyl.AddVisualShape(cyl_shape)

def update_cylinder_visual():
    pA = spring_damper.GetMarker1().GetAbsCoord().pos
    pB = spring_damper.GetMarker2().GetAbsCoord().pos

    diff = pB - pA
    length = diff.Length()

    if length < 1e-6:
        length = 1e-6
        diff = chrono.ChVectorD(0, 1, 0)

    # Compute rotation to align cylinder from origin (pA) to pB
    z_axis = chrono.ChVectorD(0, 0, 1)
    axis = chrono.Vcross(z_axis, diff)
    angle = chrono.Vdot(z_axis, diff)

    # Compute midpoint position
    midpoint = (pA + pB) * 0.5

    # Create quaternion rotation from z_axis to diff
    rot = chrono.Q_from_AngAxis(diff.GetNormalized(), 0)  # Identity rotation initially
    # But we want the shortest rotation from z_axis to diff:
    import math
    from math import acos, degrees
    dot = z_axis.Dot(diff.GetNormalized())
    if abs(dot) < 1.0:
        rot_axis = z_axis.Cross(diff.GetNormalized()).GetNormalized()
        rot_angle = math.acos(dot)
        rot = chrono.Q_from_AngAxis(rot_angle, rot_axis)
    else:
        # z_axis and diff are aligned or anti-aligned
        if dot > 0:
            rot = chrono.QIDENTITY
        else:
            rot = chrono.Q_from_AngAxis(math.pi, chrono.ChVectorD(1, 0, 0))  # 180 deg around x

    # Update cylinder position and rotation
    cyl.SetPos(midpoint)
    cyl.SetRot(rot)

    # Update cylinder endpoints to be centered on origin and length along z:
    cyl_shape.GetCylinderGeometry().p1 = chrono.ChVectorD(0, 0, -length / 2)
    cyl_shape.GetCylinderGeometry().p2 = chrono.ChVectorD(0, 0, length / 2)

# Bind update_cylinder_visual to run each step
def custom_step():
    update_cylinder_visual()

# Setup timestep and simulation loop:

application.SetTimestep(0.01)
application.SetTryRealtime(True)

application.AssetBindAll()
application.AssetUpdateAll()

while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    system.DoStepDynamics(application.GetTimestep())
    custom_step()
    application.EndScene()