import chrono as chrono
import chrono-python-irrlicht as irrlicht
import numpy as np


irrlicht.init()
chrono.init()


soil_params = chrono.SoilParams()
soil_params.density = 1800.0  
soil_params.elasticity = 1.0e9  
soil_params.poisson_ratio = 0.3
soil_params.stiffness = 1.0e6  
soil_params.viscosity = 1.0e4  


hmmwv_params = chrono.RigidBodyParams()
hmmwv_params.mass = 5000.0  
hmmwv_params.elasticity = 1.0e9  
hmmwv_params.poisson_ratio = 0.3
hmmwv_params.stiffness = 1.0e6  
hmmwv_params.viscosity = 1.0e4  


terrain_params = chrono.SCMParams()
terrain_params.width = 100.0  
terrain_params.length = 100.0  
terrain_params.soil_density = 1800.0  
terrain_params.soil_elasticity = 1.0e9  
terrain_params.soil_poisson_ratio = 0.3
terrain_params.soil_stiffness = 1.0e6  
terrain_params.soil_viscosity = 1.0e4  


terrain = chrono.CreateSCM(terrain_params)
terrain.SetName('Terrain')


hmmwv = chrono.CreateRigidBody(hmmwv_params)
hmmwv.SetName('HMMWV')
hmmwv.SetPosition(np.array([0.0, 0.0, 0.0]))  
hmmwv.SetOrientation(np.array([0.0, 0.0, 0.0]))  
hmmwv.SetMass(5000.0)  


terrain.Add(hmmwv)


tire_params = chrono.RigidBodyParams()
tire_params.mass = 100.0  
tire_params.elasticity = 1.0e9  
tire_params.poisson_ratio = 0.3
tire_params.stiffness = 1.0e6  
tire_params.viscosity = 1.0e4  

tire = chrono.CreateRigidBody(tire_params)
tire.SetName('Tire')
tire.SetPosition(np.array([0.0, 0.0, 0.0]))  
tire.SetOrientation(np.array([0.0, 0.0, 0.0]))  
tire.SetMass(100.0)  


hmmwv.Add(tire)


hmmwv.SetMeshFilename('hmmwv.stl')
terrain.SetMeshFilename('terrain.stl')
tire.SetMeshFilename('tire.stl')


terrain.EnableMovingPatch()


driver = chrono.CreateInteractiveDriver()
driver.SetName('Driver')
driver.SetSteeringGain(0.1)
driver.SetThrottleGain(0.1)
driver.SetBrakeGain(0.1)


irrlicht.set_window_title("HMMWV on SCM Terrain")
irrlicht.set_target_fps(50)


while True:
    chrono.StepSimulation(0.01)
    irrlicht.draw()
    irrlicht.update()
    if irrlicht.is_key_pressed(irrlicht.KEY_ESCAPE):
        break


chrono.Close()
irrlicht.shutdown()