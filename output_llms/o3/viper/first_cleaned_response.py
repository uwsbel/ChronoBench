import math
import pychrono             as chrono
import pychrono.vehicle      as veh
import pychrono.irrlicht     as irr




chrono.SetChronoDataPath(chrono.GetChronoDataPath())          

system      = chrono.ChSystemSMC()                            
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))              





terrain = veh.RigidTerrain(system)

patch_mat = chrono.ChMaterialSurfaceSMC()
patch_mat.SetFriction(0.9)            
patch_mat.SetRestitution(0.01)
patch_mat.SetYoungModulus(2e7)

patch = terrain.AddPatch(
            patch_mat,
            chrono.ChVectorD(0, 0, 0),                       
            chrono.ChVectorD(0, 0, 1),                       
            60, 60)                                          
patch.SetTexture(chrono.GetChronoDataFile("terrain/textures/grass.jpg"), 200, 200)
terrain.Initialize()





init_loc  = chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.4))   
rover     = veh.Viper(system)
rover.Initialize(init_loc)


powertrain_speed = 2.0  






class RampSteeringDriver:
    
    def __init__(self, rover, base_speed, target_steer=0.8, ramp_time=5.0):
        self.rover         = rover
        self.base_speed    = base_speed
        self.target_steer  = max(0.0, min(abs(target_steer), 1.0))  
        self.ramp_time     = max(ramp_time, 1e-6)
        self.start_time    = 0.0

    def initialize(self, time):
        self.start_time = time

    def synchronize(self, time):
        
        dt   = time - self.start_time
        frac = min(dt / self.ramp_time, 1.0)
        s    = frac * self.target_steer      
        
        left_speed  =  self.base_speed * (1.0 + s)
        right_speed =  self.base_speed * (1.0 - s)
        self.rover.SetWheelMotorSpeed(left_speed,  veh.SIDE.LEFT,  powertrain=False)
        self.rover.SetWheelMotorSpeed(right_speed, veh.SIDE.RIGHT, powertrain=False)

driver = RampSteeringDriver(rover, base_speed=powertrain_speed,
                            target_steer=0.8, ramp_time=6.0)
driver.initialize(system.GetChTime())





application = irr.ChIrrApp(system,
                           "Viper rover – rigid terrain",
                           irr.dimension2du(1280, 720))

application.AddTypicalLogo()
application.AddSkyBox()
application.AddTypicalLights(chrono.ChVectorD(60, 60, 100),   
                             chrono.ChVectorD(0, 0, 0),       
                             300)                             

application.AddChaseCamera(rover.GetChassisBody(),            
                           chrono.ChVectorD(0, 0, 1.0),       
                           5.0,                               
                           0.5)                               
application.SetTimestep(1e-3)                                 
application.AssetBindAll()
application.AssetUpdateAll()





sim_step    = 1e-3                              
real_clock  = chrono.ChRealtimeStepTimer()
end_time    = 20.0                              

print("Starting simulation … (ESC to quit)")
while application.GetDevice().run():

    time = system.GetChTime()
    if time > end_time:
        break

    
    driver.synchronize(time)

    
    application.BeginScene()
    application.DrawAll()
    application.EndScene()

    
    system.DoStepDynamics(sim_step)
    real_clock.Spin(sim_step)