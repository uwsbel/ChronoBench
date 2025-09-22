import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

class SCMTerrainParameters:
    def __init__(self, soil_parameters, soil_parameters_2, soil_parameters_3, soil_parameters_4, soil_parameters_5, soil_parameters_6):
        self.soil_parameters = soil_parameters
        self.soil_parameters_2 = soil_parameters_2
        self.soil_parameters_3 = soil_parameters_3
        self.soil_parameters_4 = soil_parameters_4
        self.soil_parameters_5 = soil_parameters_5
        self.soil_parameters_6 = soil_parameters_6

    def set_soil_parameters(self, soil_parameters):
        self.soil_parameters = soil_parameters

    def set_soil_parameters_2(self, soil_parameters_2):
        self.soil_parameters_2 = soil_parameters_2

    def set_soil_parameters_3(self, soil_parameters_3):
        self.soil_parameters_3 = soil_parameters_3

    def set_soil_parameters_4(self, soil_parameters_4):
        self.soil_parameters_4 = soil_parameters_4

    def set_soil_parameters_5(self, soil_parameters_5):
        self.soil_parameters_5 = soil_parameters_5

    def set_soil_parameters_6(self, soil_parameters_6):
        self.soil_parameters_6 = soil_parameters_6

    def get_soil_parameters(self):
        return self.soil_parameters

    def get_soil_parameters_2(self):
        return self.soil_parameters_2

    def get_soil_parameters_3(self):
        return self.soil_parameters_3

    def get_soil_parameters_4(self):
        return self.soil_parameters_4

    def get_soil_parameters_5(self):
        return self.soil_parameters_5

    def get_soil_parameters_6(self):
        return self.soil_parameters_6




terrain = veh.SCMTerrain(vehicle.GetSystem())
terrain.SetSoilParameters(2e6,   
                            0,     
                            1.1,   
                            0,     
                            30,    
                            0.01,  
                            2e8,   
                            3e4    
)


terrain_params = SCMTerrainParameters(
    soil_parameters=2e6,  
    soil_parameters_2=0,   
    soil_parameters_3=1.1,  
    soil_parameters_4=0,   
    soil_parameters_5=30,   
    soil_parameters_6=0.01,  
)


terrain.SetSoilParameters(terrain_params.get_soil_parameters())
terrain.SetSoilParameters_2(terrain_params.get_soil_parameters_2())
terrain.SetSoilParameters_3(terrain_params.get_soil_parameters_3())
terrain.SetSoilParameters_4(terrain_params.get_soil_parameters_4())
terrain.SetSoilParameters_5(terrain_params.get_soil_parameters_5())
terrain.SetSoilParameters_6(terrain_params.get_soil_parameters_6())


terrain.AddMovingPatch(vehicle.GetChassisBody(), chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(5, 3, 1))


terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0, 0.1)


terrain.Initialize(20, 20, 0.02)



vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())



driver = veh.ChInteractiveDriverIRR(vis)


steering_time = 1.0  
throttle_time = 1.0  
braking_time = 0.3   
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()






print( "VEHICLE MASS: ",  vehicle.GetVehicle().GetMass())


render_steps = math.ceil(render_step_size / step_size)


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run() :
    time = vehicle.GetSystem().GetChTime()

    
    if (step_number % render_steps == 0) :
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    
    step_number += 1

    
    realtime_timer.Spin(step_size)