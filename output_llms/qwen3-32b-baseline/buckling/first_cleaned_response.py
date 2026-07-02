import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as irr
import math


system = chrono.ChSystemNSC()
system.Set_Gravity(chrono.ChVectorD(0, 0, 0))  


system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
system.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_LINEARIZED)


mesh = fea.ChMesh()


section = fea.ChBeamSectionEulerAdvanced()
section.SetYoungModulus(2e11)  
section.SetGshearModulus(2e11 / (2 * (1 + 0.3)))  
section.SetDensity(8000)  
section.SetAsRectangularSection(0.01, 0.005)  


length = 1.0
n_elements = 20
nodes = []
for i in range(n_elements + 1):
    x = (i / n_elements) * length
    node = fea.ChNodeFEAxyzD(chrono.ChVectorD(x, 0, 0), chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 0))
    mesh.AddNode(node)
    nodes.append(node)


for i in range(n_elements):
    element = fea.ChElementBeamEuler()
    element.SetNodes(nodes[i], nodes[i + 1])
    element.SetSection(section)
    mesh.AddElement(element)


system.Add(mesh)



ground_body = chrono.ChBody()
ground_body.SetBodyFixed(True)
system.AddBody(ground_body)

fixed_constraint = chrono.ChLinkLockLock()
fixed_constraint.Initialize(nodes[0].GetBody(), ground_body, chrono.ChCoordsysD(nodes[0].GetPos()))
system.AddLink(fixed_constraint)


motor = chrono.ChLinkMotorLinearPosition()
motor.SetName("Axial Motor")
motor.SetMode(chrono.ChLinkMotorLinearPosition.POSITION_MODE)
motor.Initialize(nodes[-1].GetBody(), ground_body, chrono.ChFrameD(nodes[-1].GetPos()))


ramp_func = chrono.ChFunction_Ramp(0.0, 0.05)
motor.Set_pos_func(ramp_func)
system.AddLink(motor)


application = irr.ChIrrApp(system, "Beam Buckling Simulation", irr.dimension2du(800, 600))
application.SetSkyBox()
application.SetCamera(chrono.ChVectorD(0, 1, 5), chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 1, 0))
application.SetShowInfos(True)
application.AddLogo()
application.AddLightWithShadow(chrono.ChVectorD(10, 10, 10), chrono.ChVectorD(0, 0, 0), 5, 20, 10, 40, 512)
application.AddTypicalCamera(chrono.ChVectorD(0, 1, 5), chrono.ChVectorD(0, 0, 0))
application.SetVSync(True)


fea.draw_mesh(mesh, application.GetSceneManager(), False, chrono.ChColor(0.6, 0.6, 0.6))

application.AssetBindAll()
application.AssetUpdateAll()


application.SetTimestep(0.001)
application.SetStepManage(True)
application.SetRealtime(True)


while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()