import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr




chrono.SetChronoDataPath(chrono.GetChronoDataPath())   
system = chrono.ChSystemSMC()


system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
system.SetSolverMaxIterations(100)
system.SetTolForce(1e-8)
system.SetMaxPenetrationRecoverySpeed(1.0)




mesh = fea.ChMesh()


beam_length = 1.0
num_elems  = 20
beam_diam  = 0.02
E_modulus  = 2e11
nu         = 0.3
rho_beam   = 7850


beam_mat = fea.ChContinuumElastic(E_modulus, nu, rho_beam)


beam_section = fea.ChBeamSectionEuler(beam_diam, beam_diam, 0, 0, 0, beam_mat)


nodes = []
for i in range(num_elems + 1):
    x = beam_length * i / num_elems
    node = fea.ChNodeFEAxyzrot(chrono.ChVectorD(x, 0, 0))
    
    if i == 0:
        node.SetFixed(True)
    mesh.AddNode(node)
    nodes.append(node)


for i in range(num_elems):
    elem = fea.ChElementBeamEuler()
    elem.SetNodes(nodes[i], nodes[i+1])
    elem.SetSection(beam_section)
    
    elem.SetVisType(fea.ChElement.EVisType.E_VIS_SPRING)
    mesh.AddElement(elem)


system.Add(mesh)


vis_mat = chrono.ChVisualMaterial()
vis_mat.SetDiffuseColor(chrono.ChColor(0.7, 0.2, 0.2))
beam_asset = fea.ChVisualizationFEAmesh(mesh)
beam_asset.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_ELEM_BEAM_MVONM)
beam_asset.SetColorscaleMinMax(-1.0, 1.0)
beam_asset.SetSmoothFaces(True)
beam_asset.SetWireframe(False)
beam_asset.GetMesh().AddMaterial(vis_mat)
mesh.AddAsset(beam_asset)




pusher = chrono.ChBody()
pusher.SetBodyFixed(False)
pusher.SetMass(1.0)
pusher.SetInertiaXX(chrono.ChVectorD(0.01,0.01,0.01))
pusher.SetPos(chrono.ChVectorD(beam_length, 0, 0))
pusher.SetCollide(False)
system.AddBody(pusher)


link_point = chrono.ChLinkPointFrame()
link_point.Initialize(nodes[-1], pusher)
system.AddLink(link_point)




motor = chrono.ChLinkMotorLinearPosition()
motor.SetName("compressor")


motor.Initialize(pusher,           
                 system.Get_bodylist()[0],  
                 chrono.ChFrameD(chrono.ChVectorD(0,0,0),
                               chrono.QUNIT))    

y0 = 0.0
v  = -0.2   
ramp = chrono.ChFunction_Ramp(y0, v)
motor.SetMotionFunction(ramp)
system.Add(motor)




app = chronoirr.ChIrrApp(
    system, 
    "Beam Buckling with FEA", 
    chronoirr.dimension2du(1024,768)
)
app.AddTypicalSky()
app.AddTypicalLights()
app.AddTypicalCamera(chronoirr.vector3df(beam_length*0.5, -0.5, 0.5))


app.AssetBindAll()
app.AssetUpdateAll()




time_step = 1e-4
while app.GetDevice().run():
    app.BeginScene()
    app.DrawAll()
    system.DoStepDynamics(time_step)
    app.EndScene()