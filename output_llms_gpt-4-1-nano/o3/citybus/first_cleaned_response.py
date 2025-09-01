import math
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr




chrono.SetChronoDataPath(chrono.GetChronoDataPath())            
veh.SetDataPath(veh.GetDataPath())

step_size           = 1.0e-3            
render_fps          = 50                
render_steps        = math.floor(1.0/(render_fps*step_size))


init_loc            = chrono.ChVectorD(0.0, 0.0, 0.5)          
init_rot            = chrono.ChQuaternionD(1,0,0,0)            
tire_type           = veh.TireModelType.TMEASY                




sys = chrono.ChSystemNSC()
sys.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))




veh_params               = veh.CityBus()
veh_params.SetContactMethod(chrono.ChContactMethod_NSC)
veh_params.SetSystem(sys)


veh_params.SetChassisVisualizationType(veh.VisualizationType.MESH)
veh_params.SetSuspensionVisualizationType(veh.VisualizationType.PRIMITIVES)
veh_params.SetSteeringVisualizationType(  veh.VisualizationType.PRIMITIVES)
veh_params.SetWheelVisualizationType(     veh.VisualizationType.MESH)
veh_params.SetTireVisualizationType(      veh.VisualizationType.MESH)

veh_params.Initialize(init_loc, init_rot, 0.0)                 
veh_params.SetTireType(tire_type)


powertrain = veh_params.GetDrivetrain()
chassis    = veh_params.GetChassis()
steering   = veh_params.GetSteering()




terrain = veh.RigidTerrain(sys)
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch = terrain.AddPatch(patch_mat,
                         chrono.ChCoordsysD(chrono.ChVectorD(0,0,0), chrono.QUNIT),
                         600.0, 600.0)                      
patch.SetTexture( chrono.GetChronoDataFile("terrain/textures/concrete.jpg"), 60, 60)
terrain.Initialize()




app = veh.ChWheeledVehicleIrrApp(veh_params,                      
                                 "CityBus rigid-terrain demo",    
                                 irr.dimension2du(1280,720))
app.AddTypicalLighting(chrono.ChVectorD(30, -30, 120), chrono.ChVectorD(0, 0, 0), 120)
app.SetTimestep(step_size)
app.SetHUDLocation(500, 20)
app.SetChaseCamera(chrono.ChVectorD(0.0, 2.0, 1.5), 6.0, 0.5)     
app.AssetBindAll()
app.AssetUpdateAll()




driver = veh.ChIrrGuiDriver(app)

driver.SetSteeringDelta( 2.5 * chrono.CH_C_DEG_TO_RAD )          
driver.SetThrottleDelta( 0.02 )
driver.SetBrakeDelta(    0.1  )

driver.Initialize()




realtime_timer = chrono.ChRealtimeStepTimer()
step_number    = 0

print("Controls:")
print("  W/S : throttle up/down")
print("  A/D : steer left/right")
print("  SPACE: toggle hand brake")
print("  Q   : toggle debug visualization")
print("  ESC : exit")

while app.GetDevice().run():
    
    if step_number % render_steps == 0:
        app.BeginScene()
        app.DrawAll()
        app.EndScene()

    
    driver_inputs = driver.GetInputs()

    
    veh_params.GetVehicle().GetDrivetrain().Synchronize( chrono.ChTime(0), driver_inputs.m_throttle)
    veh_params.Synchronize( sys.GetChTime(),
                            driver_inputs,
                            terrain )

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    veh_params.Advance(step_size)

    sys.DoStepDynamics(step_size)

    
    realtime_timer.Spin(step_size)

    step_number += 1