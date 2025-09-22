import pychrono.core as chrono
import pychrono.irrlicht as chronoirr




chrono.SetChronoDataPath("")      
sys = chrono.ChSystemNSC()        




ground = chrono.ChBody()
ground.SetBodyFixed(True)
sys.Add(ground)




mass = chrono.ChBody()
mass.SetMass(1.0)                        
mass.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))  
mass.SetPos(chrono.ChVectorD(0, 0, -1.0))            


sphere_shape = chrono.ChSphereShape()
sphere_shape.GetSphereGeometry().rad = 0.1
mass.AddAsset(sphere_shape)

color_asset = chrono.ChColorAsset()
color_asset.SetColor(chrono.ChColor(0.2, 0.2, 1.0))   
mass.AddAsset(color_asset)

sys.Add(mass)




spring = chrono.ChLinkTSDA()



pA = chrono.ChVectorD(0, 0, 0)      
pB = chrono.ChVectorD(0, 0, 0)      


spring.Initialize(ground,            
                  mass,              
                  False,             
                  chrono.ChVectorD(0,0,0),  
                  chrono.ChVectorD(0,0,-1)) 


spring.SetSpringCoefficient(50.0)   
spring.SetDampingCoefficient(5.0)   
spring.SetRestLength(1.0)            

sys.Add(spring)




cyl = chrono.ChCylinderShape()
cyl.GetCylinderGeometry().rad = 0.02   

cyl.SetEndPoints(chrono.ChVectorD(0,0,0), chrono.ChVectorD(0,0,-1))
spring.AddAsset(cyl)




app = chronoirr.ChIrrApp(sys,
                         "Mass–Spring–Damper Demo",
                         chronoirr.dimension2du(1024, 768))


app.AddLogo()                  
app.AddSkyBox()
app.AddTypicalLights()
app.AddTypicalCamera(chronoirr.vector3df(1,1,1),  
                     chronoirr.vector3df(0,0,0))  


app.AssetBindAll()
app.AssetUpdateAll()




t_end = 5.0
dt    = 1e-3

while app.Run():
    app.BeginScene()
    app.DrawAll()
    
    pA_w = spring.GetEndPoint1World()
    pB_w = spring.GetEndPoint2World()
    cyl.SetEndPoints(pA_w, pB_w)
    app.EndScene()
    sys.DoStepDynamics(dt)