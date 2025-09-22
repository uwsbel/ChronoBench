import pychrono.core as chrono
import pychrono.irrlicht as chronoirr


chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.001)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.001)


system = chrono.ChSystemNSC()


terrain = chrono.ChBodyEasyBox(100.0, 0.1, 100.0, 3000, True, True)
terrain.SetPos(chrono.ChVectorD(0, -0.05, 0))
terrain.SetCollide(True)
system.Add(terrain)


truck = chrono.ChBody()
truck.SetMass(10000)  
truck.SetInertiaXX(chrono.ChVectorD(1000, 1000, 1000))
truck.SetPos(chrono.ChVectorD(0, 0.5, 0))  
truck.SetCollide(True)
system.Add(truck)


tire_radius = 0.5
for i in range(2):  
    wheel = chrono.ChBodyEasyCylinder(tire_radius, 0.2, 1000, True, True)
    wheel.SetPos(chrono.ChVectorD(-1.0 + i * 2.0, 0.5 - tire_radius, 0.6))
    wheel.SetCollide(True)
    system.Add(wheel)
    
    
    tire = chrono.ChTMeasy()
    tire.SetWheelRadius(tire_radius)
    tire.SetWheelWidth(0.2)
    tire.SetTireMass(10.0)
    tire.SetFriction(0.9)
    wheel.AddAsset(tire)


app = chronoirr.ChIrrApp(system, "MAN 10t Truck Simulation", chronoirr.dimension2d(800, 600))
app.AddSkyBox()
app.AddLight(chrono.ChVectorD(10, 10, 10), chrono.ChVectorD(0, 0, 0), chrono.ChColor(1, 1, 1))
app.SetChaseCamera(chrono.ChVectorD(0, 1, -5), chrono.ChVectorD(0, 0.5, 0))
app.SetShowContactPoints(True)


throttle = 0.0
steering = 0.0
braking = 0.0


while app.GetDevice().run():
    app.BeginScene()
    app.DrawAll()
    app.EndScene()

    
    throttle = 1.0  
    steering = 0.0  
    braking = 0.0   

    
    system.DoStepDynamics(0.01)


app.GetDevice().drop()