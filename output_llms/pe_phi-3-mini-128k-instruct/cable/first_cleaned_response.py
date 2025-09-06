import chrono
import chrono.fea
import chrono.irrlicht as irr


vis = chrono.irrlicht.ChVisualSystemIrrlicht()
vis.Initialize()


sys = chrono.ChSystemSMC()


chrono.ChVector3d(0, -9.81, 0)


beam_length = 10.0
beam_width = 0.1
num_elements = 100
beam_wy = beam_width / beam_length


mesh = chrono.ChMesh()
msection = chrono.ChBeamSectionEulerAdvanced()
msection.SetAsRectangularSection(beam_wy, beam_wz)
msection.SetYoungModulus(0.01e9)
msection.SetShearModulus(0.01e9 * 0.3)
msection.SetRayleighDamping(0.000)


hnode1 = chrono.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
hnode2 = chrono.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(beam_length, 0, 0)))
mesh.AddNode(hnode1)
mesh.AddNode(hnode2)


for i in range(1, num_elements):
    hnode1_next = hnode1.GetNextNode(hnode2)
    hnode2_next = hnode2.GetNextNode(hnode1)
    belement = chrono.ChElementBeamEuler()
    belement.SetNodes(hnode1, hnode2_next)
    belement.SetSection(msection)
    mesh.AddElement(belement)


beam_shape = chrono.ChVisualShapeBox(chrono.ChVector3d(beam_length, beam_width, 0.01))


vis.AddVisualizationObject(beam_shape)


sys.Add(mesh)


hinge_constraint = chrono.ChConstraintHinge2d()
hinge_constraint.SetAnchor(hnode1)
hinge_constraint.SetAxis(chrono.ChVector3d(0, 1, 0))


sys.Add(hinge_constraint)


ground_plane = chrono.ChPlane()
ground_plane.SetNormal(chrono.ChVector3d(0, 1, 0))
ground_plane.SetOrigin(chrono.ChVector3d(0, 0, 0))
sys.Add(ground_plane)


vis.AddCamera("MainCamera", chrono.vector.ChVector3d(0, 5, 10), chrono.vector.ChVector3d(0, 0, 0), chrono.vector.ChQuaternion.Degrees(-30, 0, 0, 1))


vis.AddTypicalLights()
vis.AddSkyBox()


scene_manager = chrono.irrlicht.ChSceneManager()
scene_manager.SetFontScales(64.0, 64.0, 64.0)


application = chrono.irrlicht.ChIrrApp(scene_manager, vis)


while application.GetDevice().Run():
    sys.DoStepDynamics(0.01)
    vis.BeginScene()
    vis.Clear(chrono.VizConstants.BgColor)
    vis.DrawAll()
    vis.EndScene()

    
    for node in mesh.GetNodes():
        pos = node.GetPosition()
        vis.AddActor(chrono.irr.draw.line(pos, pos + chrono.ChVector3d(0, 0, 0.01))  
    vis.AddActor(chrono.irr.draw.line(hnode1.GetPosition(), hnode2.GetPosition(), chrono.ChColor(1, 0, 0)))  

    
    application.DoStep()


vis.DestroyWindow()