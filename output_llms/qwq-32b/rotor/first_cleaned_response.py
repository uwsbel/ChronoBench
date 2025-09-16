import pychrono as chrono
import pychrono.igabase as igabase
import pychrono.postprocess as postprocess
import pychrono.irrlicht as chronoirr


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.SetPos(chrono.ChVectorD(0, 0, 0))
system.Add(ground)


beam_body = chrono.ChBody()
system.Add(beam_body)
beam_body.SetBodyFixed(False)


control_points = [
    chrono.ChVectorD(0, 0, 0),
    chrono.ChVectorD(0.5, 0, 0),
    chrono.ChVectorD(1, 0, 0)
]
knots = [0, 0, 1, 2, 2]
weights = [1, 1, 1]
degree = 1

curve = chrono.ChNurbsCurve(3, degree, control_points, knots, weights)
cross_section = chrono.ChBeamSectionRectangle(0.05, 0.05)
material = chrono.ChMaterialBeam()
material.Set_E(210e9)
material.Set_G(80e9)
material.Set_density(7800)

beam_element = chrono.iga.ChIgaBeamElement(curve, cross_section, material)
beam_body.AddElement(beam_element)


revolute = chrono.ChLinkLockRevolute()
revolute.Initialize(ground, beam_body, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
system.Add(revolute)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(ground, beam_body, False, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 0))
system.Add(motor)
motor.SetSpeed(10)  


flywheel = chrono.ChBodyEasySphere(0.1, 1000)
flywheel.SetPos(chrono.ChVectorD(0.5, 0, 0))
system.Add(flywheel)


spherical = chrono.ChLinkLockSpherical()
spherical.Initialize(beam_body, flywheel, chrono.ChFrameD(chrono.ChVectorD(0.5, 0, 0)))
system.Add(spherical)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Jeffcott Rotor Simulation')
vis.SetSymbolscale(0.002)
vis.SetShadows(True)
vis.SetLightDirection(chrono.ChVectorD(0, 0, 1))
system.SetVisualSystem(vis)


beam_body.AddAsset(chrono.ChLineShape(chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(1, 0, 0)))
flywheel.AddAsset(chrono.ChColorAsset(chrono.ChColor(0.5, 0, 0)))


post = postprocess.ChPostProcess()
post.SetPath('output/')
post.Set_fps(30)
post.SetNumFilesSaved(1000)
post.SetBytesPerVar(4)

fem_vis = postprocess.ChPostFemMeshVisualization(beam_element.GetMesh())
fem_vis.SetFEMdataType(postprocess.ChPostFemVisualization.ELEMENTS)
fem_vis.SetColorscaleMinMax(-1e6, 1e6)
post.AddVisualization(fem_vis)
post.Initialize()


application = chronoirr.ChIrrApp(system, vis)
application.AddTypicalLights()
application.AddAllVisualization()


application.SetTimestep(0.001)
application.SetTryRealtime(True)

while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    
    
    post.Export()
    
    application.DoStep()
    application.EndScene()