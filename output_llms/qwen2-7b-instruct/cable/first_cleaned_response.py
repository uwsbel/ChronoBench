fromchrono = __import__('chrono')
fromchronoimportchrono as chrono
frompychronoimportchrono as pychrono
frompychronoimportirrlichtengine
frompychronoimportchronoengine
frompychronoimportvisual
frompychronoimportutils


pychrono.init()


world = pychrono.ChSystemNSC()


default_gravity = pychrono.ChVectorD(0, -9.81, 0)


ground = pychrono.ChBodyEasyBox(1000, 1, 100, 10000)
ground.SetPos(pychrono.ChVectorD(0, -1, 0))
ground.SetBodyFixed(True)
world.Add(ground)


beam_length = 1.0
beam_width = 0.1
beam_height = 0.05


beam = pychrono.ChLinkCable()
beam.SetBeamWidth(beam_width)
beam.SetBeamHeight(beam_height)
beam.SetBeamDensity(1000)
beam.SetBeamModulus(210e9)
beam.SetBeamPoisson(0.3)
beam.SetBeamCrossSection(pychrono.ChVectorD(beam_width, beam_height, beam_height))


node1 = pychrono.ChLinkCableNode()
node1.SetPos(pychrono.ChVectorD(0, 0, 0))
beam.AddNode(node1)

node2 = pychrono.ChLinkCableNode()
node2.SetPos(pychrono.ChVectorD(beam_length, 0, 0))
beam.AddNode(node2)


beam.SetNodes(node1, node2)
world.Add(beam)


beam.SetBodyForce(default_gravity)


engine = irrlichtengine.IrrlichtEngine()
engine.AttachSystem(world)
engine.SetWindowCaption("PyChrono ANCF Beam Simulation")
engine.SetWindowSize(800, 600)
engine.SetTargetFPS(60)


while engine.IsRunning():
    engine.BeginScene()
    engine.DrawAll()
    world.DoStepDynamics(1/60.0)
    engine.EndScene()


pychrono.close()