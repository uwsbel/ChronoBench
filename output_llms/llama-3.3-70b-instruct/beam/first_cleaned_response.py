import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.beams as chronobeam


sys = chrono.ChSystemNSC()


contact_material = chrono.ChMaterialSurfaceNSC()
contact_material.SetFriction(0.5)
contact_material.SetDampingF(0.2)
contact_material.SetCompliance(1e-9)
contact_material.SetComplianceT(1e-9)
sys.SetDefaultContactMaterial(contact_material)


beam_mesh = chronobeam.ChBeamSectionEuler()
beam_mesh.SetYoungModulus(200e9)  
beam_mesh.SetArea(0.01)  
beam_mesh.SetIyy(1e-5)  
beam_mesh.SetIzz(1e-5)  
beam_mesh.SetJ(1e-6)  
beam_mesh.SetDensity(7850)  


beam = chronobeam.ChBeamSectionEuler()
beam.SetMesh(beam_mesh)
beam.SetNodes([chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(1, 0, 0)])
sys.Add(beam)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AddCamera(chrono.ChVectorD(0, 0, 5))
vis.AddTypicalLights()


while vis.Run():
    sys.DoStepDynamics(0.01)
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()