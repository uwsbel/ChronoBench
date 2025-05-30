import pychrono.core as chrono
import pychrono.irrlicht as chronoirr





system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


material = chrono.ChMaterialBeamANCF()
material.SetDensity(7800)  
material.SetYoungModulus(210e9)  
material.SetPoissonRatio(0.3)






nodes = [
    chrono.ChVectorD(0, 0, 0),
    chrono.ChVectorD(1, 0, 0),
    chrono.ChVectorD(1, 1, 0),
    chrono.ChVectorD(0, 1, 0)
]


elements = [
    [0, 1],
    [1, 2],
    [2, 3],
    [3, 0]
]


mesh = chrono.ChMesh()
for i in range(len(nodes)):
    mesh.AddNode(nodes[i])

for element in elements:
    beam = chrono.ChBeamANCF()
    beam.SetNodes(mesh.GetNode(element[0]), mesh.GetNode(element[1]))
    beam.SetMaterial(material)
    beam.SetSectionProperties(0.01, 0.01**3/12)  
    mesh.AddElement(beam)


system.Add(mesh)






mesh.GetNode(0).SetFixed(True)


force = chrono.ChForce()
force.SetMode(chrono.ChForce.FORCE)
force.SetF_x(chrono.ChFunction_Const(100))  
mesh.GetNode(2).AddForce(force)





application = chronoirr.ChIrrApp(system, "Beam FEM Example", chronoirr.dimension2du(1024, 768))
application.AddTypicalSky()
application.AddTypicalCamera(chronoirr.vector3df(0.5, 0.5, 2))
application.AssetBindAll()
application.AssetUpdateAll()


application.SetTimestep(0.001)
while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()