import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.iga as iga
import pychrono.fea as fea




system = chrono.ChSystemSMC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))




L = 1.0           
nr_elems = 20     


degree = 3
ctrl_pts = [
    chrono.ChVectorD(0.0, 0.0, 0.0),
    chrono.ChVectorD(L/3, 0.0, 0.0),
    chrono.ChVectorD(2*L/3, 0.0, 0.0),
    chrono.ChVectorD(L, 0.0, 0.0),
]

knots = [0,0,0,0, 1,1,1,1]
weights = [1,1,1,1]

curve = iga.ChNurbsCurve(degree, knots, weights, ctrl_pts)


section = iga.ChBeamSectionCosserat()
section.SetDensity(7800)              
section.SetYoungModulus(2e11)         
section.SetGwithPoissonRatio(0.3)     
section.SetAsCircularSection(0.02)    


builder = iga.ChBuilderBeamIGA(system)
builder.BuildBeam(curve, section, nr_elems,
                  iga.ChBuilderBeamIGA.BEAM_ELEMENT_TYPE_Iga)

mesh = builder.GetMesh()
system.Add(mesh)


mesh.SetAutomaticGravity(False)





node0 = mesh.GetNode(0)
pos0 = node0.GetPos()  


ground = chrono.ChBody()
ground.SetBodyFixed(True)
system.Add(ground)


marker_beam = chrono.ChMarkerFEAxyzrot(node0)
system.Add(marker_beam)


marker_ground = chrono.ChMarker()
marker_ground.SetFrame(chrono.ChFrameD(pos0, chrono.QUNIT))
ground.AddMarker(marker_ground)


link_clamp = chrono.ChLinkMateGeneric()
link_clamp.Initialize(marker_ground, marker_beam)

link_clamp.SetConstrainedCoords(True, True, True,
                                False, True, True)
system.Add(link_clamp)


motor = chrono.ChLinkMotorRotationAngleSpeed()
motor.Initialize(marker_ground, marker_beam,
                 chrono.ChFrameD(pos0, chrono.Q_from_AngAxis(chrono.CH_C_PI / 2, chrono.ChVectorD(0,1,0))))

rpm_target = 1000
omega_target = rpm_target * 2 * chrono.CH_C_PI / 60.0
f_angle = chrono.ChFunction_Ramp(0, omega_target)  
motor.SetAngleFunction(f_angle)
motor.SetSpeedFunction(chrono.ChFunction_Const(omega_target))
system.Add(motor)





num_nodes = mesh.GetNnodes()
i_mid = num_nodes // 2
node_mid = mesh.GetNode(i_mid)
pos_mid = node_mid.GetPos()


flywheel = chrono.ChBody()
flywheel.SetMass(2.0)  

Rw = 0.1
Tw = 0.02
Iw = 0.5 * flywheel.GetMass() * Rw*Rw
flywheel.SetInertiaXX(chrono.ChVectorD(Iw, 0.5*Iw, 0.5*Iw))
flywheel.SetPos(pos_mid)
flywheel.SetRot(chrono.QUNIT)
system.Add(flywheel)


marker_fw = chrono.ChMarker()
marker_fw.SetFrame(chrono.ChFrameD(pos_mid, chrono.QUNIT))
flywheel.AddMarker(marker_fw)

marker_beam_mid = chrono.ChMarkerFEAxyz(node_mid)
system.Add(marker_beam_mid)


link_fw = chrono.ChLinkMateGeneric()
link_fw.Initialize(marker_fw, marker_beam_mid)
link_fw.SetConstrainedCoords(True, True, True, True, True, True)
system.Add(link_fw)





vis_mesh = fea.ChVisualizationFEAmesh(mesh)
vis_mesh.SetWireframe(True)
vis_mesh.SetDrawUndeformed(False)
mesh.AddAsset(vis_mesh)


application = irr.ChIrrApp(
    system,                            
    "Jeffcott Rotor with IGA Beam",   
    irr.dimension2du(1024, 768)
)
application.AddTypicalLights()
application.AddLogo()
application.AddSkyBox()
application.AddCamera(irr.vector3df(0.5, -1.5, 0.5))
application.SetShowInfo(True)
application.AssetBindAll()
application.AssetUpdateAll()




application.SetTimestep(1e-3)
while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()