import pychrono

import pychrono.fea as fea

import pychrono.irrlicht as irr




chrono = pychrono.ChSystemNSC()




beam = fea.ChBeamRodNSC()

beam.SetDensity(1000.0)

beam.SetE(2.1e7)

beam.SetEA(2.1e7 / 0.01)

beam.SetG(2.1e7 / (2 * (0.01 ** 2)))

beam.SetIA(2.1e7 / (0.01 ** 4))

beam.SetL(1.0)

beam.SetRho(0.01)

beam.SetA(pychrono.ChAxial(0.01, 0.01))




node = chrono.sampleNodes(1)[0]

beam.SetNode(node)




chrono.AddBody(beam)




irr.initialize()

window = irr.createWindow()




scene_manager = irr.createScenemanager()

scene_manager.AddBody(beam)




while chrono.t < 10.0:

    chrono.DoStepDynamics(0.01)

    scene_manager.drawAll()

    irr.endScene()




irr.endWindow()

chrono.destroy()