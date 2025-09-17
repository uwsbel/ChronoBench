import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea
import pychrono.utils as utils


chrono.ChSystemNSC system


visualization = chronoirr.ChIrrApp(system, 'ANCF Beam Simulation', chronoirr.dimension2du(800, 600))


floor = chrono.ChBodyEasyBox(10, 0.1, 10, 1000, True, True, chrono.ChMaterialSurfaceNSC())
floor.SetPos(chrono.ChVectorD(0, -0.5, 0))
floor.SetBodyFixed(True)
system.Add(floor)


beam_length = 5.0
beam_width = 0.1
beam_height = 0.1
num_elements = 10


E = 210e9  
nu = 0.3   
rho = 7800  


beam_mesh = fea.ChElementCableANCF()
beam_mesh.SetLength(beam_length)
beam_mesh.SetNumElements(num_elements)
beam_mesh.SetYoungModulus(E)
beam_mesh.SetPoissonRatio(nu)
beam_mesh.SetDensity(rho)


beam = fea.ChNodeFEabs()
beam.SetPos(chrono.ChVectorD(0, 0, 0))
beam.SetBodyFixed(True)
system.Add(beam)

beam_mesh.SetNode(beam)
system.Add(beam_mesh)


beam_shape = chrono.ChVisualShapeBox(beam_width, beam_height, beam_length)
beam_shape.SetColor(chrono.ChColor(0.2, 0.2, 0.2))
beam_mesh.AddVisualShape(beam_shape)


system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


while visualization.Run():
    system.DoStepDynamics(0.01)
    visualization.BeginScene()
    visualization.Render()
    visualization.EndScene()


visualization.Close()