import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.vehicle.viper as viper_mod
import pychrono.irrlicht as irr




chrono.SetChronoDataPath('/path/to/Chrono/data/')   
system = chrono.ChSystemSMC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))




terrain = veh.RigidTerrain(system)


patch_mat = veh.RigidTerrain.PatchMaterialData()
patch_mat.mu   = 0.9
patch_mat.cr   = 0.01
patch_mat.Y    = 2e7
patch_mat.cr_mu = 0.3


terrain.AddPatch(patch_mat, chrono.CSYSNORM, 20, 10)
terrain.Initialize()





viper = viper_mod.ViperVehicle()
viper.SetContactMethod(chrono.ChMaterialSurface.SMC)
viper.SetChassisFixed(False)
viper.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0)))
viper.Initialize()


viper.SetChassisVisualizationType(veh.VisualizationType_MESH)
viper.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
viper.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
viper.SetWheelVisualizationType(veh.VisualizationType_MESH)
viper.SetTireVisualizationType(veh.VisualizationType_MESH)




class SimpleRampingDriver:
    def __init__(self, max_steering, ramp_time):
        self.max_steer = max_steering
        self.ramp_time = ramp_time
        
        self.throttle = 0.5
        self.braking  = 0.0

    def GetInputs(self, time):
        
        if time < self.ramp_time:
            steer = self.max_steer * time / self.ramp_time
        else:
            steer = self.max_steer
        
        return veh.DriverInputs(steer, self.throttle, self.braking)


driver = SimpleRampingDriver(max_steering=0.52, ramp_time=5.0)




app = veh.ChVehicleIrrApp(viper,                         
                          "Viper Rover on Rigid Terrain")  


app.AddTypicalLights()
app.AddSkyBox()
app.AddLogo()
app.SetChaseCamera(chrono.ChCoordsysD(chrono.ChVectorD(-6, 2, 0),
                                      chrono.Q_from_AngAxis(0.25, chrono.VECT_Y)))
app.AssetBindAll()
app.AssetUpdateAll()


timestep = 1.0 / 200.0
app.SetTimestep(timestep)
system.SetStep(timestep)




time = 0.0
end_time = 20.0

while app.GetDevice().run() and time < end_time:
    
    inputs = driver.GetInputs(time)

    
    viper.Synchronize(time, inputs, terrain)
    terrain.Synchronize(time)
    app.Synchronize("",  
                    {})

    
    app.BeginScene()
    app.DrawAll()
    app.EndScene()

    
    driver_inputs = inputs  
    viper.Advance(timestep)
    terrain.Advance(timestep)
    app.Advance(timestep)

    
    time += timestep