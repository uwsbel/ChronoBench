import pychrono

import pychrono.irrlicht

from pychrono.IrrIterator import IrrIterator




chrono = pychrono.ChSystemNSC()




beam1 = pychrono.ChBeamNSC()

beam1.SetBeamRadius(0.05)

beam1.SetNode(pychrono.ChNodeCosSFD(pychrono.ChCoastingODE(chrono)))

beam1.SetNode(pychrono.ChNodeCosSFD(pychrono.ChCoastingODE(chrono)))

beam1.SetNode(pychrono.ChNodeCosSFD(pychrono.ChCoastingODE(chrono)))

beam1.SetNode(pychrono.ChNodeCosSFD(pychrono.ChCoastingODE(chrono)))

beam1.SetBeamLength(1.0)

beam1.SetMass(1.0)

beam1.SetDensity(1000.0)

beam1.SetElasticModulus(2.1e11)

beam1.SetShearModulus(0.8e11)




chrono.AddBody(beam1)




irr = pychrono.irrlicht.ChIrrApp()




irr.SetUsePhongLighting(True)

irr.SetBeamsColor(pychrono.irrlicht.E_BTDF_AMBIENT_AND_DIFFUSE, 0.6, 0.6, 0.6, 1.0)




beam_visual = pychrono.irrlicht.ChBeamVisual(irr)

beam_visual.SetBeamRadius(beam1.GetBeamRadius())

beam_visual.SetBeamLength(beam1.GetBeamLength())

beam_visual.SetColor(0.6, 0.6, 0.6)

irr.AddBeamVisual(beam_visual)




while(irr.GetDevice().run()):

    chrono.DoStepDynamics(0.01)

    irr.BeginScene()

    beam_visual.Draw(irr)

    irr.EndScene()