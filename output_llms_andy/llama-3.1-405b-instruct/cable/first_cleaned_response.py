import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import numpy as np


chrono.SetChronoDataPath("/path/to/chrono/data/")


mysystem = chrono.ChSystemNSC()


ground = chrono.ChBodyEasyBox(mysystem, 10, 1, 10, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -5, 0))
ground.SetBodyFixed(True)


beam = chrono.ChANCFBeam()
beam.SetBeamLength(10)
beam.SetBeamSectionRadius(0.1)
beam.SetYoungModulus(2e7)
beam.SetBeamDensity(1000)
beam.SetBeamRayleighDamping(0.1)


mysystem.Add(beam)


hinge = chrono.ChLinkMateGeneric()
hinge.Initialize(beam, ground, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.Q_ROTATE_X_TO_Y))
hinge.SetConstrainedDOFs(True, True, True, False, True, True)
mysystem.Add(hinge)


gravity = chrono.ChForce()
gravity.SetF(chrono.ChVectorD(0, -9.81, 0))
beam.AddForce(gravity)


myapplication = chronoirr.ChIrrApp(mysystem, "PyChrono Beam Simulation", chronoirr.dimension2du(800, 600))


while myapplication.GetDevice().run():
    myapplication.BeginScene()
    myapplication.DrawAll()
    myapplication.EndScene()
    mysystem.DoStepDynamics(0.01)

    
    beam_nodes = beam.GetNodes()
    for i in range(len(beam_nodes)):
        node = beam_nodes[i]
        pos = node.GetPos()
        myapplication.GetSceneManager().addSphere(chrono.ChVectorD(pos.x(), pos.y(), pos.z()), 0.05, 10, 10, chrono.ChColor(1, 0, 0))


del mysystem
del myapplication