import pychrone as pc
import pychrone.irrlicht as irr


pc.init()


truss = pc.Truss()
truss.set_length(10)
truss.set_height(0.5)
truss.set_width(0.5)


crankshaft = pc.Cylinder()
crankshaft.set_radius(0.1)
crankshaft.set_length(1)


connecting_rod = pc.Cylinder()
connecting_rod.set_radius(0.05)
connecting_rod.set_length(1)


piston = pc.Cylinder()
piston.set_radius(0.05)
piston.set_length(0.5)


crank_joint = pc.RevoluteJoint(crankshaft, truss, [0, 0, 0], [0, 0, 0])
connecting_rod_joint = pc.RevoluteJoint(connecting_rod, crankshaft, [0, 0, 0], [0, 0, 0])
piston_joint = pc.RevoluteJoint(piston, connecting_rod, [0, 0, 0], [0, 0, 0])


pc.add_object(truss)
pc.add_object(crankshaft)
pc.add_object(connecting_rod)
pc.add_object(piston)


pc.add_joint(crank_joint)
pc.add_joint(connecting_rod_joint)
pc.add_joint(piston_joint)


motor = pc.Motor(crankshaft, [0, 0, 0], [0, 0, 1], 1)


pc.add_object(motor)


pc.set_time_step(0.01)
pc.set_gravity([0, -9.81, 0])


irr.set_camera_position([5, 5, 5])
irr.set_camera_target([0, 0, 0])
irr.set_camera_up([0, 1, 0])
irr.set_light_position([10, 10, 10])
irr.set_light_color([1, 1, 1])
irr.set_ambient_light_color([0.2, 0.2, 0.2])
irr.set_texture_path("texture.png")
irr.set_logo_path("logo.png")


pc.run()