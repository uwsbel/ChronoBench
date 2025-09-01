import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as irrlicht

# 1. Initialize system and set parameters
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, 0))  # Disable gravity

# 2. Create FEA mesh
mesh = fea.ChMesh()
system.Add(mesh)

# Beam parameters
length = 2.0    # Beam length
n_elements = 20 # Number of FEA elements
beam_width = 0.1
rho = 7800      # Density (kg/m³)
E = 2e11        # Young's modulus (Pa)
I = (beam_width**4)/12  # Area moment of inertia

# 3. Create beam section and material
beam_section = fea.ChBeamSectionEuler()
beam_section.SetDensity(rho)
beam_section.SetYoungModulus(E)
beam_section.SetAsRectangularSection(beam_width, beam_width)

# 4. Create nodes and elements
nodes = []
for i in range(n_elements + 1):
    position = chrono.ChVectorD(i * (length / n_elements), 0, 0)
    node = fea.ChNodeFEAxyzrot(chrono.ChFrameD(position))
    node.SetMass(0)
    mesh.AddNode(node)
    nodes.append(node)

for i in range(n_elements):
    element = fea.ChElementBeamEuler()
    element.SetNodes(nodes[i], nodes[i + 1])
    element.SetSection(beam_section)
    mesh.AddElement(element)

# 5. Add constraints and boundary conditions
# Fix first node
truss = chrono.ChBody()
truss.SetBodyFixed(True)
system.Add(truss)

fix = fea.ChLinkPointFrame()
fix.Initialize(nodes[0], truss)
system.Add(fix)

# 6. Create sliding end constraint with motor
slider = chrono.ChBodyEasyBox(0.1, 0.1, 0.1, 1000)
slider.SetPos(chrono.ChVectorD(length, 0, 0))
system.Add(slider)

motor_link = chrono.ChLinkMotorLinearPosition()
motor_link.Initialize(slider, truss, 
                     chrono.ChFrameD(chrono.ChVectorD(length, 0, 0), 
                                    chrono.Q_ROTATE_Y_TO_X))
system.Add(motor_link)

# Custom motor function (ramp displacement)
motor_func = chrono.ChFunction_Ramp(0, 0.01)  # 0.01 m/s displacement rate
motor_link.SetMotorFunction(motor_func)

# Connect slider to last node
node_slider_link = fea.ChLinkPointFrame()
node_slider_link.Initialize(nodes[-1], slider)
system.Add(node_slider_link)

# 7. Set up visualization
vis = irrlicht.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Beam Buckling Simulation')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(1.5, 0.5, 2), chrono.ChVectorD(1.5, 0, 0))
vis.AddTypicalLights()

# FEA visualization settings
beam_visual = fea.ChVisualShapeFEA(mesh)
beam_visual.SetFEMdataType(fea.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
beam_visual.SetColorscaleMinMax(-1000, 1000)
beam_visual.SetSmoothFaces(True)
mesh.AddVisualShapeFEA(beam_visual)

# 8. Configure solver and timestepper
system.SetSolverType(chrono.ChSolver.Type_MINRES)
system.SetSolverMaxIterations(200)
system.SetSolverTolerance(1e-12)
system.SetTimestepperType(chrono.ChTimestepper.Type_HHT)
hht_stepper = system.GetTimestepper().AsTimestepperHHT()
hht_stepper.SetAlpha(-0.2)
hht_stepper.SetStepControl(True)
hht_stepper.SetVerbose(False)

# 9. Simulation loop
dt = 0.001
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(dt)