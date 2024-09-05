import pychrono as chrono
import pychrono.irrlicht as chronoirr


chrono.SetChronoDataPath("../../../data/")


system = chrono.ChSystemNSC()


system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


mesh = chrono.ChMesh()


nodeA = chrono.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
nodeB = chrono.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD(2, 0, 0)))
nodeC = chrono.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD(4, 0, 0)))


mesh.AddNode(nodeA)
mesh.AddNode(nodeB)
mesh.AddNode(nodeC)


msection = chrono.ChBeamSectionEulerAdvanced()
beam_wy = 0.1
beam_wz = 0.2
msection.SetAsRectangularSection(beam_wy, beam_wz)
msection.SetYoungModulus(0.01e9)
msection.SetGshearModulus(0.01e9 * 0.3)
msection.SetBeamRaleyghDamping(0.000)


beamAB = chrono.ChElementBeamEuler()
beamAB.SetNodes(nodeA, nodeB)
beamAB.SetSection(msection)

beamBC = chrono.ChElementBeamEuler()
beamBC.SetNodes(nodeB, nodeC)
beamBC.SetSection(msection)


mesh.AddElement(beamAB)
mesh.AddElement(beamBC)


system.Add(mesh)


application = chronoirr.ChIrrApp(system, "Beam FEA Demo", chronoirr.dimension2du(1024, 768))
application.AddTypicalLogo()
application.AddTypicalSky()
application.AddTypicalLights()
application.AddTypicalCamera(chronoirr.vector3df(0, 3, -3))


mvisualizebeamA = chrono.ChVisualShapeBeam(beamAB, 0.01, 20, 0.01)
mvisualizebeamA.SetMaterial(chrono.ChVisualMaterial(chrono.ChColor(0.5, 0.8, 0.8)))
beamAB.AddVisualShape(mvisualizebeamA)

mvisualizebeamC = chrono.ChVisualShapeBeam(beamBC, 0.01, 20, 0.01)
mvisualizebeamC.SetMaterial(chrono.ChVisualMaterial(chrono.ChColor(0.5, 0.8, 0.8)))
beamBC.AddVisualShape(mvisualizebeamC)


application.AssetBindAll()
application.AssetUpdateAll()


time_step = 0.01
end_time = 10.0


while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()

    
    nodeC.SetForce(chrono.ChVectorD(0, -2, 0))

    
    application.DoStep()

    
    if system.GetChTime() > end_time:
        break

    application.EndScene()


application.GetDevice().drop()