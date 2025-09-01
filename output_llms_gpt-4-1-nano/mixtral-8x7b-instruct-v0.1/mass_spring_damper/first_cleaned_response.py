from pychrono.core import *
from pychrono.irrlicht import *
from pychrono.fea import *


my_application = ChronoEngine()


my_visualization = IrrlichtVisualization(my_application, "Mass Spring Damper", 800, 600)
my_visualization.addLight(ChVectorD(2, -3, 0), 0.6, 0.6, 0.6)
my_visualization.addCamera(ChVectorD(3, 4, 5), ChVectorD(0, 0, 0))


mass_body = RigidBody(my_application, "mass", 1.0, ChVectorD(0, 0, 0))
mass_body.setMass(1.0)
mass_body.setInertiaXX(ChVectorD(0.08333333333333333, 0.08333333333333333, 0.08333333333333333))
my_application.addBody(mass_box)


mass_shape = SphericalShell(1.0, 16, 16)
mass_body.addVisualShape(mass_shape, 1.0, ChColor(0.8, 0.8, 0.8))


ground_body = RigidBody(my_application, "ground", 0.0, ChVectorD(0, 0, 0))
my_application.addBody(ground_body)


ground_shape = ChPlaneShape()
ground_body.addVisualShape(ground_shape, 0.1, ChColor(0.2, 0.2, 0.2))


mass_spring_damper = ChLinkTSDA(mass_body, ground_body, ChVectorD(0, 0, 0), ChVectorD(0, 0, -1))
my_application.addLink(mass_spring_damper)


mass_spring_damper.set_spring_k(50.0)
mass_spring_damper.set_damper_c(2.0)


my_application.set_timestep(0.01)
my_application.doSimulation(1000)


my_visualization.run()