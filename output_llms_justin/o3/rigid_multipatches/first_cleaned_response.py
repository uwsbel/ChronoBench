import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh




chrono.SetChronoDataPath(chrono.GetChronoDataPath())          
veh.SetDataPath(veh.GetDataPath())                            
chrono.SetChronoMallocVerbose(False)


step_size     = 2e-3        
tire_step     = step_size   
render_step   = 1.0 / 60    
sim_time      = 0.0




system = chrono.ChSystemNSC()




app = veh.ChVehicleIrrApp(system, "HMMWV – multi-patch terrain", irr.dimension2du(1280, 720))
app.AddTypicalLights()
app.SetSkyBox()           
app.SetShadows()
app.SetTimestep(step_size)





init_pos   = chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1.0), chrono.QUNIT)  
init_fwd   = chrono.ChVectorD(1, 0, 0)

fourWD     = True
engine     = veh.PowertrainModelType_SIMPLE
drivetrain = veh.DrivelineType_AWD

vehicle = veh.HMMWV_Full(system,
                         fixed=False,
                         driveType=drivetrain,
                         brakeType=veh.BrakeType_SIMPLE,
                         steeringType=veh.SteeringType_PITMAN_ARM,
                         contactMethod=chrono.ChContactMethod.NSC)
vehicle.Initialize(init_pos, init_fwd)
vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_MESH)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)


app.SetChaseCamera(vehicle.GetChassisBody(), 6.0, 0.5, 0.0)




terrain = veh.RigidTerrain(system)


patch_conc = terrain.AddPatch(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT),
                              chrono.ChVectorD(200, 5, 0.2))
patch_conc.SetContactFrictionCoefficient(0.9)
patch_conc.SetMaterialSurface(chrono.ChVehicleUtils.CreateBoxMaterial(chrono.ChContactMethod.NSC, 0.9, 0.1))
patch_conc.SetTexture(chrono.GetChronoDataFile("terrain/textures/concrete.jpg"), 200, 5)


patch_grass = terrain.AddPatch(chrono.ChCoordsysD(chrono.ChVectorD(0, 6, 0), chrono.QUNIT),
                               chrono.ChVectorD(200, 5, 0.2))
patch_grass.SetContactFrictionCoefficient(0.8)
patch_grass.SetElevation(0.0)
patch_grass.SetTexture(chrono.GetChronoDataFile("terrain/textures/grass.jpg"), 200, 5)


bump_mesh = chrono.ChTriangleMeshConnected()
bump_mesh.LoadWavefrontMesh(chrono.GetChronoDataFile("vehicle/hmmwv/bump_mesh.obj"), False, True)
bump_mesh.Transform(chrono.ChVectorF(10, -5, 0), chrono.ChMatrix33F(1))
patch_bump = terrain.AddPatch(bump_mesh, chrono.ChVectorD(10, -5, 0), chrono.ChQuaternionD(1, 0, 0, 0))
patch_bump.SetContactFrictionCoefficient(0.6)
patch_bump.SetTexture(chrono.GetChronoDataFile("terrain/textures/concrete_bump.jpg"), 1.0, 1.0)


hmap_file   = chrono.GetChronoDataFile("terrain/heightmaps/test64.bmp")  
hmap_scale  = chrono.ChVectorD(1.0, 1.0, 0.5)    
size_x      = 64 * hmap_scale.x
size_y      = 64 * hmap_scale.y
base        = chrono.ChCoordsysD(chrono.ChVectorD(-size_x*0.5, -20, 0), chrono.QUNIT)
patch_hmap  = terrain.AddPatch(base, hmap_file, hmap_scale, size_x, size_y, 0.0)
patch_hmap.SetContactFrictionCoefficient(0.75)
patch_hmap.SetTexture(chrono.GetChronoDataFile("terrain/textures/dirt.jpg"), size_x, size_y)

terrain.Initialize()

app.AssetBindAll()
app.AssetUpdateAll()




driver = veh.ChIrrGuiDriver(app)
driver.SetSteeringDelta(0.04)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.1)
driver.Initialize()




realtime_timer = chrono.ChRealtimeStepTimer()
render_accum   = 0.0




print("Controls:")
print("  W/S : throttle")
print("  A/D : steer")
print("  SPACE: brake")
print("  P   : toggle powertrain info")

while app.GetDevice().run():

    
    if (sim_time - render_accum) >= render_step:
        app.BeginScene()
        app.DrawAll()
        app.EndScene()
        render_accum = sim_time

    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(sim_time)
    terrain.Synchronize(sim_time)
    vehicle.Synchronize(sim_time,
                        driver_inputs,
                        terrain)
    app.Synchronize("HMMWV demo", driver_inputs)

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    app.Advance(step_size)

    
    sim_time += step_size
    realtime_timer.Spin(step_size)