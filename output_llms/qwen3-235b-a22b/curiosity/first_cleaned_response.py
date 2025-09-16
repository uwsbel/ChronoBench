import pychrono as chrono
import pychrono.irrlicht as irrlicht
import pychrono.vehicle as vehicle


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.SetCollide(True)


ground_collision = ground.GetCollisionModel()
ground_collision.ClearModel()
ground_collision.AddBox(100, 1, 100)  
ground_collision.BuildModel()
ground.SetMaterialSurface(chrono.ChMaterialSurfaceNSC())
ground.GetMaterialSurface(0).SetFriction(0.9)


ground_box = chrono.ChBoxShape()
ground_box.GetBoxGeometry().Size = chrono.ChVectorD(100, 1, 100)
ground.AddAsset(ground_box)

ground_texture = chrono.ChTexture()
ground_texture.SetTextureFilename(chrono.GetChronoDataFile("textures/concrete.jpg"))
ground.AddAsset(ground_texture)

system.Add(ground)


rover = vehicle.ChRoverCuriosity()
rover.Initialize(system, chrono.ChCoordsysD(chrono.ChVectorD(0, 2, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
system.Add(rover)


app = irrlicht.ChIrrApp(system, 'Curiosity Rover Simulation', irrlicht.dimension2du(1280, 720))
app.AddTypicalLights()
app.AddSkyBox()
app.AddCamera(chrono.ChVectorD(0, 5, -10), chrono.ChVectorD(0, 0, 0))
app.AssetBindAll()
app.AssetUpdateAll()


app.GetSceneManager().getVideoDriver().setTextureFilter(irrlicht.ETFT_ANISOTROPIC)
app.EnableShadows()


guienv = app.GetDevice().getGUIEnvironment()
logo = guienv.addImage(
    irrlicht.rect(10, 10, 110, 110),
    0, -1, irrlicht.SColor(255, 255, 255, 255),
    irrlicht.core.stringw("Chrono Logo")
)


while app.GetDevice().run():
    
    keymap = app.GetDevice().getEventReceiver().getKeyMap()
    steering = 0.0
    if keymap[irrlicht.KEY_LEFT]:
        steering = -1.0
    elif keymap[irrlicht.KEY_RIGHT]:
        steering = 1.0
    
    
    rover.SetSteering(steering)
    
    
    system.DoStepDynamics(1e-3)
    
    
    app.BeginScene()
    app.DrawAll()
    app.EndScene()