import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as irr


chrono.SetChronoDataPath('PYCHRONO_DATA_DIR')
my_system = chrono.ChSystemNSC()


mesh = fea.ChMesh()


node1 = fea.ChNodeFEAxyz(chrono.ChVectorD(0, 0, 0))
node2 = fea.ChNodeFEAxyz(chrono.ChVectorD(1, 0, 0))
mesh.AddNode(node1)
mesh.AddNode(node2)


beam = fea.ChElementBeamIGA()
beam.SetNodes(node1, node2)
beam.SetYoungModulus(73e9)
beam.SetGwithPoissonRatio(0.3)
beam.SetBeamRaleyghDamping(0.005)
mesh.AddElement(beam)


my_system.Add(mesh)




my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


application = irr.ChIrrApp(my_system, 'Beam FEA simulation', irr.dimension2du(1024, 768))


application.AssetBindAll()
application.AssetUpdateAll()
application.SetTimestep(0.01)
application.SetTryRealtime(True)

while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()