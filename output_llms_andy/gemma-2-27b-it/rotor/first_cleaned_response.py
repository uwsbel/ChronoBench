import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as irr





mysystem = chrono.ChSystemNSC()


mysystem.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))






beam_length = 1.0
beam_radius = 0.05
beam_material = fea.ChMaterialShellANCF(rho=7850, E=200e9, nu=0.3)


nurbs_geometry = fea.ChNURBSGeomPatch()



beam_element = fea.ChElementShellANCF(nurbs_geometry, beam_material)


beam = fea.ChBeamIGA(beam_element)


beam.SetPos(chrono.ChVectorD(0, 0, 0))
beam.SetRot(chrono.Q_from_AngAxis(chrono.CH_C_PI / 2, chrono.ChVectorD(0, 0, 1)))


mysystem.Add(beam)






flywheel_radius = 0.2
flywheel_mass = 10


flywheel = chrono.ChBodyEasyCylinder(flywheel_radius, flywheel_radius, flywheel_mass)


flywheel.SetPos(beam.GetPos() + chrono.ChVectorD(beam_length / 2, 0, 0))
chrono.ChLinkMateFix(flywheel, beam)


mysystem.Add(flywheel)






motor = chrono.ChBody()
motor.SetPos(chrono.ChVectorD(0, 0, 0))


motor_joint = chrono.ChLinkRevolute()
motor_joint.Initialize(motor, beam, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0)))
motor_joint.SetSpeedFunction(chrono.ChFunction_Const(10))


mysystem.Add(motor)
mysystem.Add(motor_joint)






vis = irr.ChIrrApp(mysystem, "Jeffcott Rotor Simulation", irr.dimension2du(800, 600))
vis.AddTypicalSky()
vis.AddTypicalLights()
vis.AddCamera(chrono.ChVectorD(2, 2, 2))


vis.SetFEMVisualization(True)


vis.SetTimestep(0.01)
vis.Run()