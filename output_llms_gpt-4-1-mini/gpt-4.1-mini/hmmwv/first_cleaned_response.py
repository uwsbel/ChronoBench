import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr




chrono.SetChronoDataPath(chrono.GetChronoDataPath())


contact_method = chrono.ChContactMethod_NSC  



initLoc = chrono.ChVectorD(0, 0, 0.6)  
initRot = chrono.ChQuaternionD(1, 0, 0, 0)


hmmwv = veh.HmmwvVehicle(contact_type=contact_method, tire_model=veh.HmmwvVehicle.TireModelType_TMEASY)


hmmwv.SetInitPosition(chrono.ChFrameD(initLoc, initRot))


hmmwv.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetTireVisualizationType(veh.VisualizationType_PRIMITIVES)


hmmwv.Initialize()




terrain = veh.RigidTerrain(hmmwv.GetSystem())
terrain.SetPlane(0, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngY(0)))  


patch_mat = chrono.ChMaterialSurfaceSMC()
patch_mat.SetFriction(0.9)
terrain.SetContactSurfaceMaterial(patch_mat)


patch_length = 150
patch_width = 150

patch = terrain.AddPatch(patch_mat,
                         chrono.ChVectorD(0, 0, 0),
                         chrono.ChVectorD(patch_length, 0, 0),
                         chrono.ChVectorD(0, patch_width, 0))
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 25, 25)

terrain.Initialize()




application = veh.ChWheeledVehicleIrrApp(hmmwv, "HMMWV on Rigid Terrain - TMEASY Tires", irr.dimension2du(1280, 720))

application.SetTimestep(0.02)  
application.AddTypicalLights()
application.AddLogo()


application.Initialize()
application.SetCameraPosition(chrono.ChVectorD(8, -18, 5))
application.SetCameraTarget(hmmwv.GetChassis().GetPos())


driver = veh.ChWheeledVehicleVisualSystemIrrlicht(hmmwv.GetSystem())

driver = application.GetDriver()
driver.SetInputDelay(0.1)  
driver.Initialize()


timestep = 0.02
step_per_render = 1


while application.GetDevice().run():
    time = hmmwv.GetSystem().GetChTime()

    
    driver_inputs = driver.GetInputs()

    
    hmmwv.SteeringSystem().SetInput(driver_inputs.m_steering)
    hmmwv.SetThrottle(driver_inputs.m_throttle)
    hmmwv.SetBrake(driver_inputs.m_braking)

    
    application.BeginScene()
    application.DrawAll()

    hmmwv.Advance(timestep)
    terrain.Advance(timestep)
    driver.Advance(timestep)

    application.EndScene()

    
    
    chrono.ChRealtimeStepTimer().Advance(timestep)