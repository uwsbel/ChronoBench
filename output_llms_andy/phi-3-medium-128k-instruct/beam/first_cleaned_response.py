import pychrono

import pychrono.irrlicht

from pychrono.IrrIterator import IrrIterator




chrono = pychrono.ChNo




beam = pychrono.ChBeamDef()

beam.SetBody("BeamBody")

beam.SetWeld("WeldPoint")

beam.SetBeamRadius(0.1)

beam.SetBeamLength(1.0)

beam.SetBeamHeight(0.2)

beam.SetBeamDensity(7850.0)

beam.SetBeamElasticModulus(2.1e11)

beam.SetBeamPoissonRatio(0.3)




system = pychrono.ChSystemNSC()




beam_body = pychrono.ChBodyVect(pychrono.Vec3(0, 0, 0), 1000.0, beam)

system.AddBody(beam__body)




irr_license = pychrono.irrlicht.chrono_irrlicht_license_d.aLicense()

window = pychrono.irrlicht.ChIrrApp(&irr_license)

window.SetUse3DScroll(True)

window.SetUserDrawModeCallback(IrrIterator)




beam_mesh = pychrono.ChMeshCreateBox(0.2, 0.2, 1.0)

beam_body.AddMesh(beam_mesh)




system.Set_Verbose(True)

while(window.GetDevice()->run()):

    system.DoTimeStep()

    window.DoMainLoop()


print("Simulation completed.")