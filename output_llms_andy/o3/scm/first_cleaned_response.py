import math
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr


chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")





RENDER_FPS        = 50            
RENDER_STEP_SIZE  = 1.0 / RENDER_FPS

STEP_SIZE         = 1e-3          

TERRAIN_LENGTH    = 80.0          
TERRAIN_WIDTH     = 80.0          
NODE_SPACING      = 0.1           


INIT_LOC          = chrono.ChVectorD(0.0, 0.0, 0.5)          
INIT_ROT          = chrono.ChQuaternionD(1, 0, 0, 0)          
TIRE_STEP_SIZE    = STEP_SIZE                                 




hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
hmmwv.SetPowertrainType(veh.PowertrainType_SHAFTS)
hmmwv.SetDriveType(veh.DriveType_AWD)
hmmwv.SetTireType(veh.TireType_RIGID)
hmmwv.SetTireStepSize(TIRE_STEP_SIZE)


hmmwv.SetChassisVisualizationType     (veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType  (veh.VisualizationType_MESH)
hmmwv.SetSteeringVisualizationType    (veh.VisualizationType_MESH)
hmmwv.SetWheelVisualizationType       (veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType        (veh.VisualizationType_MESH)

hmmwv.Initialize()

system = hmmwv.GetSystem()        




terrain = veh.SCMDeformableTerrain(system)
terrain.SetPlane(chrono.ChCoordsysd(chrono.ChVectorD(0,0,0), chrono.QUNIT))
terrain.Initialize(TERRAIN_LENGTH, TERRAIN_WIDTH, NODE_SPACING)


Bekker_Kphi     = 5301e3     
Bekker_Kc       = 102e3      
Bekker_n        = 1.1        
Mohr_cohesion   = 1.0e3      
Mohr_friction   = 30.0       
Janosi_shear    = 0.02       

terrain.SetSoilParameters(Bekker_Kphi, Bekker_Kc, Bekker_n,
                          Mohr_cohesion, Mohr_friction, Janosi_shear)


terrain.SetElasticModulus(2e8)          
terrain.SetDamping(0.01)                


terrain.AddMovingPatch(hmmwv.GetChassisBody(),
                       chrono.ChVectorD( 0, 0, 0),
                       chrono.ChVectorD( 5, 5, 1))     


terrain.SetPlotType(veh.SCMDeformableTerrain.PLOT_SINKAGE, 0, 0.2)




app = veh.ChWheeledVehicleIrrApp(hmmwv, "HMMWV on SCM Deformable Terrain",
                                 irr.dimension2du(1280, 720))

app.SetSkyBox()
app.AddTypicalLogo()
app.AddTypicalLights(irr.vector3df(30, 100, 30),     
                     irr.vector3df(-30, 100, 30),    
                     250.0, 130.0)
app.AddTypicalCamera(irr.vector3df(1,1,1))           
app.SetChaseCamera(chrono.ChVectorD(0, 0, 0.5), 6.0, 0.5)


driver = veh.ChIrrGuiDriver(app,  
                            0.3,  
                            0.01, 
                            0.3)  
driver.Initialize()


realtime_timer = chrono.ChRealtimeStepTimer()




render_steps = math.ceil(RENDER_STEP_SIZE / STEP_SIZE)
step_counter = 0

print("Simulation running.  Use arrow keys (steer), SPACE (brake), and")
print("keys W/S for throttle.  Close the Irrlicht window to exit.")

while app.GetDevice().run():
    
    if step_counter % render_steps == 0:
        app.BeginScene(True, True, irr.SColor(255, 140, 161, 192))
        app.DrawAll()
        app.EndScene()

    
    time = system.GetChTime()
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    hmmwv.Synchronize(time, driver_inputs, terrain)
    app.Synchronize("", driver_inputs)

    
    driver.Advance(STEP_SIZE)
    terrain.Advance(STEP_SIZE)
    hmmwv.Advance(STEP_SIZE)
    app.Advance(STEP_SIZE)

    
    realtime_timer.Spin(STEP_SIZE)

    step_counter += 1