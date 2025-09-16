import pychrono as chrono
import pychrono.irr as irr


chrono.init()


world = chrono.World()


engine = irr.IrrlichtEngine()


beam_length = 1.0  
beam_width = 0.1  
beam_height = 0.2  


beam_geometry = chrono.Shape(chrono.ShapeType.BOX)
beam_geometry.set_size(beam_length, beam_width, beam_height)


beam_material = chrono.Material()
beam_material.set_density(1000.0)  
beam_material.set_elasticity(100.0)  
beam_material.set_friction(0.1)


beam_nodes = [
    chrono.Node(beam_geometry, chrono.Position(0.0, 0.0, 0.0)),
    chrono.Node(beam_geometry, chrono.Position(beam_length, 0.0, 0.0))
]


beam_elements = [
    chrono.Element(beam_nodes[0], beam_nodes[1], beam_material)
]


world.add_object(beam_geometry)


world.set_gravity(chrono.Vector3(0.0, -9.81, 0.0))


for node in beam_nodes:
    node.set_position(chrono.Vector3(node.get_position().x, node.get_position().y, node.get_position().z))





engine.run()


engine.draw()