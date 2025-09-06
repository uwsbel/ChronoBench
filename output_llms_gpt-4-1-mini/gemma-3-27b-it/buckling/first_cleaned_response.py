import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea
import math


sys = chrono.ChSystemSMC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


beam_length = 1.0  
beam_height = 0.1  
beam_width = 0.05  
youngs_modulus = 2.0e11  
shear_modulus = 8.0e10  
density = 7850  
damping = 0.01


mesh = fea.ChMesh()


num_elements = 10
node_spacing = beam_length / num_elements

for i in range(num_elements + 1):
    x = i * node_spacing
    node = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(x, 0, 0)))
    mesh.AddNode(node)


for i in range(num_elements):
    node1 = mesh.GetNode(i)
    node2 = mesh.GetNode(i + 1)
    element = fea.ChElementBeamEuler()
    element.SetNodes(node1, node2)
    
    
    section = fea.ChBeamSectionEulerAdvanced()
    section.SetAsRectangularSection(beam_height, beam_width)
    section.SetYoungModulus(youngs_modulus)
    section.SetShearModulus(shear_modulus)
    section.SetRayleighDamping(damping)

    element.SetSection(section)
    mesh.AddElement(element)


body = chrono.ChBody()
body.SetPos(chrono.ChVector3d(0, 0, 0))
body.SetMass(density * beam_height * beam_width * beam_length)
body.AddAsset(mesh)
body.EnableCollision(True)
sys.Add(body)



fixed_point = chrono.ChVector3d(0, 0, 0)
body.SetFixedPoint(fixed_point)


force_magnitude = 1000  
force_point = chrono.ChVector3d(beam_length, 0, 0)
force_direction = chrono.ChVector3d(0, -1, 0)  


force_sensor = chrono.ChForceSensor()
force_sensor.SetForceFunction(chrono.ChFunction_Const(chrono.ChVector3d(0, -force_magnitude, 0)))
force_sensor.SetPoint(force_point)
body.AddForce(force_sensor)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Beam Buckling Simulation')
vis.Initialize()
vis.AddCamera(chrono.ChVector3d(0, 2, -3))
vis.AddTypicalLights()


time_step = 0.01
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(time_step)