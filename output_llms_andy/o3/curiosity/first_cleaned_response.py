from math import pi
import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh




time_step        = 1.0e-3           
tire_step_size   = 1.0e-3           
render_step_size = 1.0 / 60.0       




chrono.SetChronoDataPath( chrono.GetChronoDataPath() )
veh.SetDataPath   ( veh.GetDataPath() )

system = chrono.ChSystemNSC()          


system.Set_G_acc( chrono.ChVectorD(0, 0, -3.71) )   




terrain = veh.RigidTerrain( system )

patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction( 0.9 )
patch_mat.SetRestitution( 0.01 )


patch = terrain.AddPatch( patch_mat,
                          chrono.ChCoordsysD( chrono.ChVectorD( 0, 0, 0 ),
                                              chrono.QUNIT ),
                          10, 10 )


texture_file = chrono.GetChronoDataFile( "vehicle/terrain/textures/sand.jpg" )
patch.SetTexture( texture_file, 1.0, 1.0 )

terrain.Initialize()




rover = veh.Curiosity( system )          

init_loc = chrono.ChVectorD( 0, 0, 0.35 )   
rover.Initialize( chrono.ChCoordsysD(init_loc, chrono.QUNIT) )


rover.SetStepsize( tire_step_size )




class KeyboardDriver(veh.ChDriver):
    def __init__(self, app) -> None:
        super().__init__()
        self.throttle = 0.0
        self.steering = 0.0
        self.braking  = 0.0
        self.app      = app
        self.wireframe = False

    
    def OnEvent(self, ev):
        if ev.EventType != chronoirr.EET_KEY_INPUT_EVENT:
            return False

        key = ev.KeyInput
        down = key.PressedDown

        if   key.Key == chronoirr.KEY_KEY_W:  self.throttle =  1.0 if down else 0
        elif key.Key == chronoirr.KEY_KEY_S:  self.throttle = -1.0 if down else 0
        elif key.Key == chronoirr.KEY_KEY_A:  self.steering =  1.0 if down else 0
        elif key.Key == chronoirr.KEY_KEY_D:  self.steering = -1.0 if down else 0
        elif key.Key == chronoirr.KEY_SPACE:  self.braking  =  1.0 if down else 0
        elif key.Key == chronoirr.KEY_TAB and not down:
            self.wireframe = not self.wireframe
            self.app.SetWireframe(self.wireframe)
        elif key.Key == chronoirr.KEY_ESCAPE and not down:
            self.app.GetDevice().closeDevice()

        return False

    
    def Synchronize(self, time):
        
        self.m_steering = 0.5 * self.m_steering + 0.5 * self.steering
        self.m_throttle = self.throttle
        self.m_braking  = self.braking

driver = KeyboardDriver(None)   




app = chronoirr.ChIrrApp( system,
                          "Curiosity rover – rigid terrain",
                          chronoirr.dimension2du(1280, 720) )
driver.app = app                                            


chassis_body = rover.GetChassis().GetBody()
cam_pos = chrono.ChVectorD( 3, -4, 2 )
app.AddChaseCamera( chassis_body, cam_pos, 1.6, 0.2 )


app.AddTypicalLogo()
app.AddTypicalSky()
app.AddLightWithShadow( chrono.ChVectorD( 5, 5, 5 ),
                        chrono.ChVectorD( 0, 0, 0 ),
                        20, 1, 20, 30, 50 )
app.AddLight( chrono.ChVectorD(-3, -3, 5), 3.0 )

app.AssetBindAll()
app.AssetUpdateAll()


app.GetDevice().setEventReceiver( driver )




realtime_timer = chrono.ChRealtimeStepTimer()
render_time    = 0.0

print("Controls: W/S throttle  A/D steer  SPACE brake  TAB wireframe")

while app.GetDevice().run():
    
    app.BeginScene()
    app.DrawAll()

    
    time_now = system.GetChTime()

    
    driver.Synchronize(time_now)

    
    rover.Synchronize( time_now,
                       driver.GetSteering(),
                       driver.GetThrottle(),
                       driver.GetBraking(),
                       terrain )

    
    step = realtime_timer.SuggestSimulationStep( time_step )
    rover.Advance( step )
    terrain.Advance( step )
    system.DoStepDynamics( step )

    
    render_time += step
    if render_time > render_step_size:
        render_time = 0.0        

    app.EndScene()


print("Simulation finished.")