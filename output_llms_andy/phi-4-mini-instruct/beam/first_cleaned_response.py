from PyChrono import *


PyChronoInit()


sys = PhysicsSystem()


beam = Beam(0.01, 0.01, 0.001, 0.001, 0.001)
beam.setElasticity(210000000)
beam.setDensity(7850)
beam.setYoungsModulus(210000000000)

node = Node(0.01, 0.01, 0.001, 0.001, 0.001)
node.setMass(1.0)


mesh = BeamMesh(beam, node)


sys.add(mesh)


sys.setGravity(0, -9.81, 0)


vis = IrrlichtViewer(sys, 800, 600)


while vis.isRunning():
    vis.update()
    sys.step(1.0 / 60.0)


PyChronoFinalize()