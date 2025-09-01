import pychrono as chrono
import pychrono.irrlicht as chronoirr




rest_length   = 1.5
spring_coef   = 50.0
damping_coef  = 1.0
step_size     = 1e-3




sys = chrono.ChSystemNSC()
sys.Set_G_acc(chrono.ChVectorD(0, 0, 0))       




ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.SetCollide(False)
sys.Add(ground)


sph = chrono.ChSphereShape()
sph.GetSphereGeometry().rad = 0.1
ground.AddAsset(sph)




def make_box(name, position, color_rgb):
    body = chrono.ChBody()
    body.SetPos(position)
    body.SetMass(1.0)
    body.SetInertiaXX(chrono.ChVectorD(1, 1, 1))
    body.SetCollide(False)

    box = chrono.ChBoxShape()
    box.GetBoxGeometry().Size = chrono.ChVectorD(0.5, 0.5, 0.5)
    box.SetColor(chrono.ChColor(*color_rgb))
    body.AddAsset(box)

    sys.Add(body)
    return body




body_1 = make_box("body_1", chrono.ChVectorD(-1.0, -3.0, 0.0), (0.6, 0.0, 0.0))
body_2 = make_box("body_2", chrono.ChVectorD( 0.0, -3.0, 0.0), (0.0, 0.6, 0.0))
body_3 = make_box("body_3", chrono.ChVectorD( 1.0, -3.0, 0.0), (0.0, 0.0, 0.6))




def make_tsda(name, bodyA, bodyB, posA, posB):
    tsda = chrono.ChLinkTSDA()
    tsda.Initialize(bodyA, bodyB, True, posA, posB)
    tsda.SetRestLength(rest_length)
    tsda.SetSpringCoefficient(spring_coef)
    tsda.SetDampingCoefficient(damping_coef)

    
    tsda_visual = chrono.ChSpringShape()
    tsda_visual.SetRadius(0.05)
    tsda.AddAsset(tsda_visual)

    sys.AddLink(tsda)
    return tsda


spring_1 = make_tsda("spring_1", body_1, ground,
                     chrono.ChVectorD(0, 0, 0),   
                     chrono.ChVectorD(-1, 0, 0))  


spring_2 = make_tsda("spring_2", body_1, body_2,
                     chrono.ChVectorD(0, 0, 0),
                     chrono.ChVectorD(0, 0, 0))


spring_3 = make_tsda("spring_3", body_2, body_3,
                     chrono.ChVectorD(0, 0, 0),
                     chrono.ChVectorD(0, 0, 0))




chrono.AssetBindAll(sys)
chrono.AssetUpdateAll(sys)




vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("TSDA chain demo")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 0, 6))
vis.AddTypicalLights()




while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    sys.DoStepDynamics(step_size)