from pychron import *

from pychron.physics import Mass, SpringDamper
from pychron.irrlicht import IrrlichtVisualizer, IrrlichtPhysics
from pychron.physics.visuals import Visual, SphereVisual, CylinderVisual



chrono = Chrono()



engine = IrrlichtPhysics(chrono)



ground = ChronoSphere(chrono, 1.0, 0.0, 0.0, 0.0, 0.0)



mass = Mass(chrono, 1.0, 0.0, 0.0, 0.0, 0.0)



spring_damper = SpringDamper(chrono, 100.0, 0.1)



spring_damper.create_link(mass, ground, ChLinkTSDA(100.0, 0.1))



mass_visual = SphereVisual(chrono, 0.2, mass.get_shape())

spring_visual = CylinderVisual(chrono, 0.1, spring_damper.get_shape())



visualizer = IrrlichtVisualizer(chrono)

visualizer.add_visual(mass_visual)

visualizer.add_visual(spring_visual)



visualizer.set_light_position(0, 10, 0)

visualizer.set_camera_position(0, 10, 10)

visualizer.set_camera_lookat(0, 0, 0)



while True:

    chrono.run()

    visualizer.render()