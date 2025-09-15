importchrono
import math


system =chrono.ChSystem()
system.SetSolverType(chrono.ChSolverType_SOR)
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


class BeamElementDemo:
    def __init__(self, system):
        self.system = system
        self.nodes = []
        self.elements = []
        
    def create_nodes(self):
        
        node1 = chrono.ChNodeFEAxyz()
        node1.SetPos(chrono.ChVectorD(0, 0, 0))
        self.system.AddNode(node1)
        self.nodes.append(node1)
        
        node2 = chrono.ChNodeFEAxyz()
        node2.SetPos(chrono.ChVectorD(2, 0, 0))
        self.system.AddNode(node2)
        self.nodes.append(node2)
        
    def create_elements(self):
        
        young_modulus = 2.1e11  
        shear_modulus = 8.1e10  
        area = 0.01  
        inertia = 0.0001  
        
        
        beam = chrono.ChElementBeam()
        beam.SetNodes(self.nodes[0], self.nodes[1])
        beam.SetYoungModulus(young_modulus)
        beam.SetShearModulus(shear_modulus)
        beam.SetArea(area)
        beam.SetMomentOfInertia(inertia)
        self.system.AddElement(beam)
        self.elements.append(beam)
        
    def add_visualization(self):
        
        for element in self.elements:
            shape = chrono.ChShape()
            shape.SetPos(element.GetNodes()[0].GetPos())
            shape.SetScale(chrono.ChVectorD(1, 1, 1))
            
            cylinder = chrono.ChCylinder()
            cylinder.SetHeight(2)
            cylinder.SetRadius(0.1)
            shape.GetAsset().AddGeometry(cylinder)
            
            element.GetAssets().push_back(shape)
            self.system.AddAsset(shape)


beam_demo = BeamElementDemo(system)
beam_demo.create_nodes()
beam_demo.create_elements()
beam_demo.add_visualization()


floor = chrono.ChBody()
floor.SetBodyFixed(True)
floor.SetPos(chrono.ChVectorD(0, -1, 0))
floor.GetCollisionModel().AddPlane(chrono.ChVectorD(0,1,0), chrono.ChVectorD(0,0,0))
floor.GetVisualShape().SetMaterial(chrono.ChVisualMaterial())
floor.GetVisualShape().SetTexture(chrono.GetChronoDataPath() + 'textures/concrete.jpg')
system.AddBody(floor)


light = chrono.ChLight()
light.SetPos(chrono.ChVectorD(0, 5, 5))
light.SetIntensity(1)
system.AddLight(light)


renderer = chrono.ChIrrRenderer()
renderer.AttachSystem(system)
renderer.SetWindowTitle('Beam Finite Element Demo')
renderer.SetCameraPosition(chrono.ChVectorD(3, 3, 3))
renderer.SetCameraRotation(chrono.ChVectorD(-60, -60, 0))
renderer.SetLightPosition(chrono.ChVectorD(0, 5, 5))
renderer.EnableShadows()
renderer.EnableAntiAliasing()


renderer.Begin()
while renderer.Run():
    renderer.Render()
    system.DoStepDynamics(0.001)
    if chrono.ChKey pressed 'q':
        break
renderer.End()