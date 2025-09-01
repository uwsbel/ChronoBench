import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as irr






time_step = 1e-3


output_dir = "BEAM_BUCKLING_FEA"





system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))





mesh = fea.ChMesh()
system.Add(mesh)


beam_length = 1.0
beam_width = 0.02
beam_height = 0.04


material = fea.ChMaterialShellANCF()
material.SetDensity(7800)
material.SetYoungModulus(2.0e11)
material.SetPoissonRatio(0.3)


section = fea.ChBeamSectionAdvanced()
section.SetAsRectangularSection(beam_width, beam_height)
section.SetMaterial(material)


num_elements = 20


nodes = []
for i in range(num_elements + 1):
    node = fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD(i * beam_length / num_elements, 0, 0)))
    nodes.append(node)
    mesh.AddNode(node)


elements = []
for i in range(num_elements):
    element = fea.ChElementBeamANCF()
    element.SetNodes(nodes[i], nodes[i + 1], nodes[i], nodes[i + 1])  
    element.SetSection(section)
    elements.append(element)
    mesh.AddElement(element)





truss_fixed = chrono.ChBody()
system.Add(truss_fixed)
truss_fixed.SetBodyFixed(True)

constraint_bc = chrono.ChLinkLockLock()
constraint_bc.Initialize(truss_fixed, nodes[0], chrono.ChCoordsysD())
system.Add(constraint_bc)





load_force = chrono.ChForce()
nodes[-1].AddForce(load_force)
load_force.SetMode(chrono.ChForce.FORCE)
load_force.SetF_x(chrono.ChFunction_Ramp(0, -1000))  





vis = irr.ChIrrApp(
    system, "Beam Buckling Simulation", irr.dimension2du(1024, 768), irr.VerticalDir_Z
)
vis.AddTypicalSky()
vis.AddTypicalLights()
vis.AddTypicalCamera(chrono.ChVectorD(beam_length / 2, -0.2, 0), chrono.ChVectorD(beam_length / 2, 0, 0))
vis.AssetBindAll()
vis.AssetUpdateAll()





system.SetSolverType(chrono.ChSolver.Type_MINRES)
system.SetSolverMaxIterations(100)
system.SetMaxPenetrationRecoverySpeed(1.0)
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

stepper = chrono.ChTimestepperHHT(system)
stepper.SetStepControl(True)
stepper.SetMinStepSize(time_step / 10)
stepper.SetMaxStepSize(time_step)
stepper.SetMode(chrono.ChTimestepperHHT.ACCELERATION)
stepper.SetScalingFactor(1.0)





while vis.GetDevice().run():
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()

    stepper.DoStep(time_step)