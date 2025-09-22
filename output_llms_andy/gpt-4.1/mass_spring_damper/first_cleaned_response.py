import pychrono as chrono
import pychrono.irrlicht as chronoirr


chrono.SetChronoDataPath(chrono.GetChronoDataPath())


system = chrono.ChSystemNSC()




ground = chrono.ChBody()
ground.SetBodyFixed(True)
system.Add(ground)


mass = chrono.ChBodyEasyBox(0.2, 0.2, 0.2,  
                            1.0,            
                            True,           
                            True)           
mass.SetPos(chrono.ChVectorD(0, 0.5, 0))
system.Add(mass)



pos_ground = chrono.ChVectorD(0, 0, 0)
pos_mass = chrono.ChVectorD(0, -0.1, 0)  

spring = chrono.ChLinkTSDA()
spring.Initialize(ground, mass, False, pos_ground, mass.GetPos() + pos_mass)


spring.SetSpringCoefficient(100)
spring.SetDampingCoefficient(5)
spring.SetRestLength((pos_ground - (mass.GetPos() + pos_mass)).Length())

system.AddLink(spring)




mass.SetPos_dt(chrono.ChVectorD(0, 0, 0))




app = chronoirr.ChVisualSystemIrrlicht()
app.AttachSystem(system)
app.SetWindowSize(1024, 768)
app.SetWindowTitle("PyChrono: Mass-Spring-Damper (ChLinkTSDA)")
app.Initialize()
app.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
app.AddSkyBox()
app.AddCamera(chrono.ChVectorD(1.0, 0.7, 1.0), chrono.ChVectorD(0, 0.3, 0))
app.AddTypicalLights()


spring_shape = chrono.ChLinkTSDAVisualShape()
spring_shape.SetColor(chrono.ChColor(0.2, 0.6, 0.8))
spring_shape.SetResolution(80)
spring_shape.SetRadius(0.03)
spring.AddVisualShape(spring_shape)


mass.GetVisualShape(0).SetColor(chrono.ChColor(0.8, 0.2, 0.2))


app.AssetBindAll()
app.AssetUpdateAll()

timestep = 0.001

while app.Run():
    app.BeginScene()
    app.Render()
    app.EndScene()
    system.DoStepDynamics(timestep)