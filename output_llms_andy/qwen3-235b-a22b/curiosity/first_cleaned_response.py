import pychrono as chrono
import pychrono.irrlicht as chronoirr


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.SetCollide(True)
ground.GetCollisionModel().ClearModel()
ground.GetCollisionModel().AddBox(100, 1, 100)  
ground.SetPos(chrono.ChVectorD(0, -1, 0))
ground.GetMaterialSurface().SetFriction(0.8)
ground.GetMaterialSurface().SetRestitution(0.1)
system.Add(ground)


ground_texture = chrono.ChTexture()
ground_texture.SetTextureFilename(chrono.GetChronoDataPath() + 'textures/concrete.jpg')
ground.AddAsset(ground_texture)


rover = chrono.ChBody()
rover.SetMass(100)
rover.SetInertiaXX(chrono.ChVectorD(1, 1, 1))
rover.SetPos(chrono.ChVectorD(0, 0.5, 0))
rover_vis = chrono.ChBoxShape(chrono.ChVectorD(0.5, 0.25, 1), chrono.ChColor(0.6, 0.6, 0.6))
rover.AddAsset(rover_vis)
system.Add(rover)


class Driver:
    def __init__(self, app):
        self.app = app
        self.steering = 0.0
        self.throttle = 0.0

    def Update(self):
        
        if self.app.KeyDown(chronoirr.KEY_UP):
            self.throttle += 0.01
        elif self.app.KeyDown(chronoirr.KEY_DOWN):
            self.throttle -= 0.01
        if self.app.KeyDown(chronoirr.KEY_LEFT):
            self.steering += 0.05
        elif self.app.KeyDown(chronoirr.KEY_RIGHT):
            self.steering -= 0.05
        
        self.throttle = max(-1, min(1, self.throttle))
        self.steering = max(-1, min(1, self.steering))


app = chronoirr.ChIrrApp(system, 'Curiosity Rover Simulation', chronoirr.dimension2du(1024, 768))
app.AddLogo(chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')
app.AddSkyBox()
camera = app.GetSceneManager().getActiveCamera()
app.SetTimestep(0.01)


app.AddLightWithShadow(chronoirr.vector3df(10, 20, 10), chronoirr.vector3df(0, 0, 0), 90, 0.2, 100, 100, chronoirr.SColorf(1,1,1))
app.AddShadowToAll()


driver = Driver(app)


while app.GetDevice().run():
    
    driver.Update()
    
    
    torque = chrono.ChVectorD(0, driver.steering * 10, 0)
    force = chrono.ChVectorD(driver.throttle * 100, 0, 0)
    rover.Empty_forces()
    rover.Accumulate_torque(torque, True)
    rover.Accumulate_force(force, rover.GetPos(), True)
    
    
    system.DoStepDynamics(app.GetTimestep())
    
    
    rover_pos = rover.GetPos()
    camera.setPosition(chronoirr.vector3df(rover_pos.x, rover_pos.y + 3, rover_pos.z - 6))
    camera.setTarget(chronoirr.vector3df(rover_pos.x, rover_pos.y, rover_pos.z))
    
    
    app.BeginScene()
    app.DrawAll()
    app.EndScene()