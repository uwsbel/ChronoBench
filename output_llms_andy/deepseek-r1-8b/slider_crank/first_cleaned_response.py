import pychrono as pychrono
import pychrono.visvis as visvis
from pychrono.objects import *
from pychrono.gear import *
from pychrono.mechanisms import *
from pychrono.materials import *
from pychrono.utils import *


pychrono.init()


physics = pychrono.PyPhysicsServer(timestep=1e-5)


scene = Scene()
renderer = visvis.IrrlichtSceneRenderer(window_size=(800, 600))


light = DirectionalLight(position=(5, 5, 5), color=(1, 1, 1))
renderer.add_light(light)


camera = PerspectiveCamera(position=(5, 5, 5), look_at=(0, 0, 0), fov=45)
renderer.add_camera(camera)



crankshaft_texture = visvis.load_texture("crankshaft.png")

connecting_rod_texture = visvis.load_texture("connecting_rod.png")

piston_texture = visvis.load_texture("piston.png")

logo_texture = visvis.load_texture("logo.png")


floor = Beam(name="floor", point1=Point(pos=[0, 0, 0]), point2=Point(pos=[10, 0, 0]), material=Material(mass=0, friction=0))
physics.add_object(floor)


crankshaft = Beam(name="crankshaft", point1=Point(pos=[0, 0, 0]), point2=Point(pos=[0, 0, 0]), 
                direction=Vector3d(0, 0, 1), length=1, material=Material(mass=2, friction=0.1))
physics.add_object(crankshaft)


connecting_rod = Beam(name="connecting_rod", point1=Point(pos=[0, 0, 0]), point2=Point(pos=[0, 0, 0]),
                       direction=Vector3d(1, 0, 0), length=5, material=Material(mass=0.5, friction=0.1))
physics.add_object(connecting_rod)


piston = Box(name="piston", size=Vector3d(1, 1, 0.5), pos=Point(pos=[0, 0, 0]), material=Material(mass=1, friction=0.1))
physics.add_object(piston)


fixed_joint = RevoluteJoint(name="crankshaft_base", 
                          body1=floor,
                          body2=crankshaft,
                          anchor1=Point(pos=[0, 0, 0]),
                          anchor2=Point(pos=[0, 0, 0]),
                          axis=Vector3d(0, 0, 1),
                          rotation=0)
physics.add_joint(fixed_joint)


fixed_joint2 = RevoluteJoint(name="connecting_rod_base",
                          body1=crankshaft,
                          body2=connecting_rod,
                          anchor1=Point(pos=[0, 0, 0]),
                          anchor2=Point(pos=[0, 0, 0]),
                          axis=Vector3d(1, 0, 0),
                          rotation=0)
physics.add_joint(fixed_joint2)


fixed_joint3 = RevoluteJoint(name="piston_base",
                          body1=connecting_rod,
                          body2=piston,
                          anchor1=Point(pos=[0, 0, 0]),
                          anchor2=Point(pos=[0, 0, 0]),
                          axis=Vector3d(0, 1, 0),
                          rotation=0)
physics.add_joint(fixed_joint3)


motor = Motor(name="crankshaft_motor",
              body=crankshaft,
              joint=crankshaft.s joint,
              torque=100,
              angular_speed=1.0,
              max_torque=100)
physics.add_object(motor)



crankshaft.mesh().mesh().texture = crankshaft_texture

connecting_rod.mesh().mesh().texture = connecting_rod_texture

piston.mesh().mesh().texture = piston_texture


logo = Plane(name="logo", size=Vector2d(0.5, 0.5), pos=Point(pos=[800, 10, 5]), 
             material=Material(tex=logo_texture, tex_scale=(0.5, 0.5)))
physics.add_object(logo)


physics.set_fixed_timestep(1e-5)
physics.start()


renderer.set_camera(camera)
renderer.set_light(light)


while True:
    physics.update()
    renderer.render()
    
    
    
    
    pass