import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr




chrono.SetChronoDataPath(chrono.GetChronoDataFile(""))          
veh.SetDataPath(chrono.GetChronoDataFile("vehicle/"))           




system          = chrono.ChSystemSMC()          
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))


step_size       = 2.0e-3                        
render_fps      = 60.0                          
render_steps    = int(1.0/(render_fps*step_size))




terrain         = veh.RigidTerrain(system)

mat_terr        = chrono.ChMaterialSurfaceSMC()
mat_terr.SetFriction(0.9)
mat_terr.SetRestitution(0.01)
mat_terr.SetYoungModulus(2e7)

patch_dimX      = 250.0     
patch_dimY      = 250.0     
patch_thick     = 0.1       

patch = terrain.AddPatch(
    mat_terr,
    chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT),
    patch_dimX,
    patch_dimY,
    patch_thick
)


patch.SetTexture(chrono.GetChronoDataFile("terrain/textures/tile4.jpg"),
                 tex_scale_x=patch_dimX*0.6,
                 tex_scale_y=patch_dimY*0.6)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.8))
terrain.Initialize()




init_pos        = chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.40),
                                     chrono.QUNIT)

vehicle         = veh.Sedan(system,
                            fixed=False,
                            brake_type=veh.BrakeType.SHAFTS,
                            contact_method=chrono.ChContactMethod_SMC)

vehicle.SetName("BMW_E90")
vehicle.Initialize(init_pos)


vehicle.SetChassisVisualizationType   (veh.VisualizationType.MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType.PRIMITIVES)
vehicle.SetSteeringVisualizationType  (veh.VisualizationType.PRIMITIVES)
vehicle.SetWheelVisualizationType     (veh.VisualizationType.MESH)


tire_vis = veh.VisualizationType.MESH

for axle in vehicle.GetAxles():
    for wheel in axle.m_wheels:
        wheel_index = wheel.GetId()
        side        = "L" if wheel_index % 2 == 0 else "R"
        axle_id     = int(wheel_index/2)
        label       = f"TMeasy_A{axle_id}_{side}"
        tire        = veh.TMeasyTire(label)
        tire.SetVisualizationType(tire_vis)
        tire.Initialize(wheel)




app = veh.ChWheeledVehicleIrrApp(vehicle,
                                 "BMW E90 sedan – TMeasy tires (interactive)",
                                 irr.dimension2du(1280, 720))


app.SetSkyBox()
app.AddTypicalLogo()
app.AddTypicalLights(chrono.ChVector3df( 150, 100, 200),
                     chrono.ChVector3df(-150,-100, 200),
                     250, 130)
app.SetChaseCamera(chrono.ChVectorD(0.0, 0.5, 0.0), 6.0, 0.4)   

app.Initialize()


driver = veh.ChIrrGuiDriver(app)
driver.SetSteeringDelta(0.035)      
driver.SetThrottleDelta(0.02)       
driver.SetBrakingDelta (0.08)       
driver.Initialize()




step_number = 0
while app.GetDevice().run():

    time = system.GetChTime()

    
    if step_number % render_steps == 0:
        app.BeginScene(True, True, chrono.SColor(255, 140, 161, 192))
        app.DrawAll()
        app.EndScene()

    
    driver_inputs = driver.GetInputs()        

    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    app.Synchronize("BMW E90 demo", driver_inputs)

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    app.Advance(step_size)

    
    system.DoStepDynamics(step_size)

    step_number += 1