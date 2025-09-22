import chrono as ch
import chrono.irrlicht as irr
import chrono.vehicle as veh


ch.ChEngine.Initialize()


app = irr.ChIrrApp()


vehicle = veh.ChBMW_E90()


vehicle.SetVisualization(visible=True, 
                        collide=False, 
                        contact_vis=False, 
                        render_manifold=False)


terrain = veh.ChTerrain()
terrain.SetType(veh.ChTerrain.Type.RIGID)


tire_model = veh.ChTMEasyTire(vehicle, terrain)


driver = veh.ChInteractiveDriver(vehicle)


vehicle.SetPosition(ch.ChVector3d(0, 0, 1))
vehicle.SetHeading(0)


vehicle_node = irr.ChIrrNode(vehicle.GetChassisBody())
app.Add(vehicle_node)


camera = irr.ChChaseCamera(vehicle_node, 
                            ch.ChVector3d(0, -10, 5), 
                            ch.ChVector3d(0, 0, 1))


light = irr.ChDirectionalLight(ch.ChVector3d(1, 1, 1), 
                                ch.ChVector3d(-1, -1, -1))


skybox = irr.ChSkyBox(app.GetSceneManager())


terrain.SetSurfaceTexture("textures/terrain.jpg")
terrain.SetLogoTexture("textures/logo.png")


while app.Run():
    
    vehicle.Update(0.01)
    terrain.Update(0.01)

    
    app.BeginScene()
    app.AddCamera(camera)
    app.AddLight(light)
    app.Render()
    app.EndScene()

    
    ch.ChEngine.Advance(0.01)