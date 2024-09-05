import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.irrlicht.irrlicht as irr


chrono.init()


world = chrono.World()


irr_engine = irr.IrrlichtEngine()


scene = irr_engine.createScene()


beam_length = 1.0  
beam_width = 0.1  
beam_material = chrono.Material(chrono.Material.Type.STEEL)  


beam_elements = []
for i in range(10):  
    element = chrono.BeamElement(beam_length, beam_width, beam_material)
    beam_elements.append(element)


beam = chrono.Beam(beam_elements)


beam.set HingePoint(chrono.Point(0, 0, 0))


beam.setGravity(chrono.Vector(0, -9.81, 0))


ground = chrono.Ground()


ground.setPosition(chrono.Vector(0, 0, 0))


world.add(ground)


world.add(beam)


dt = 0.01  
iterations = 1000  


for i in range(iterations):
    
    beam.update(dt)

    
    scene.render()


world.clear()
irr_engine.shutdown()