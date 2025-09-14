import pychrono as chrono
import pychrono.irrlicht as chrono_irrlicht
import pychrono.fea as fea

# 1. Initialize system and set gravity
system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

# 2. Create mesh container
mesh = chrono.ChMesh()
system.Add(mesh)

# 3. Create beam material and section properties
beam_material = chrono.ChBeamMaterialEuler()
beam_material.SetDensity(1000)        # kg/m^3
beam_material.SetYoungModulus(1e7)    # Pa
beam_material.SetShearModulus(beam_material.GetYoungModulus()/(2*(1+0.3)))  # Poisson ratio 0.3
beam_material.SetRayleighDampingBeta(0.01)

beam_section = chrono.ChBeamSectionEuler()
beam_section.SetAsCircularSection(0.05)  # Radius 5cm
beam_section.SetMaterial(beam_material)

# 4. Create nodes and beam elements
num_elements = 10
length = 5.0  # meters
node_spacing = length / num_elements

nodes = []
for i in range(num_elements + 1):
    position = chrono.ChVector3d(i * node_spacing, 0, 0)
    node = fea.ChNodeFEAxyzrot(chrono.ChFrame(position))
    node.SetMass(0)
    mesh.AddNode(node)
    nodes.append(node)

# Fix first node
nodes[0].SetFixed(True)

# Create elements between nodes
for i in range(num_elements):
    beam_element = fea.ChElementBeamEuler()
    beam_element.SetNodes(nodes[i], nodes[i+1])
    beam_element.SetSection(beam_section)
    mesh.AddElement(beam_element)

# 5. Set up visualization
vmesh = chrono.ChVisualShapeFEA(mesh)
vmesh.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vmesh.SetBeamResolution(chrono.ChVisualShapeFEA.BeamResolution_High)
vmesh.SetBeamWidth(0.02)
vmesh.SetColor(chrono.ChColor(0.9, 0.7, 0.3))
mesh.AddVisualShapeFEA(vmesh)

# 6. Configure solver and integrator for FEA
system.SetSolverType(chrono.ChSolver.Type_MINRES)
system.GetSolverAsIterative().SetMaxIterations(100)
system.GetSolverAsIterative().SetTolerance(1e-10)
system.SetTimestepperType(chrono.ChTimestepper.Type_HHT)
integrator = system.GetTimestepper().AsTimestepperHHT()
integrator.SetAlpha(-0.2)
integrator.SetMaxIters(5)
integrator.SetAbsTolerances(1e-5)

# 7. Set up Irrlicht visualization
app = chrono_irrlicht.ChIrrApp(system, "Beam FEM Demo", chrono_irrlicht.dimension2du(1280, 720))
app.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
app.AddSkyBox()
app.AddTypicalLights()
app.AddCamera(chrono.ChVector3d(-1, 0.5, 3), chrono.ChVector3d(3, 0, 0))
app.AssetBindAll()
app.AssetUpdateAll()

# 8. Configure time stepping and run simulation
time_step = 0.001
app.SetTimestep(time_step)

while app.GetDevice().run():
    app.BeginScene()
    app.DrawAll()
    app.DoStep()
    app.EndScene()