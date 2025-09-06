import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

def main():
    
    system = chrono.ChSystemSMC()
    
    
    application = irr.ChIrrApp(system, 'HMMWV on Deformable Terrain', irr.CHIRR.dimension2du(1280, 720))
    application.SetCamera(chrono.Camera(chrono.ChVectorD(0, 10, 30), chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 1, 0)))
    application.AddTypicalLights()
    application.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    
    
    hmmwv = veh.HMMWV()
    hmmwv.SetChassisCollisionType(veh.ChassisCollisionType_NONE)
    hmmwv.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1), chrono.QUNIT))
    hmmwv.Initialize()
    
    
    hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
    
    
    soil = veh.ChDeformableSoil()
    soil.SetShearModulus(1e6)
    soil.SetCohesion(1e4)
    soil.SetFrictionAngle(chrono.CH_C_PI/6)
    soil.SetDensity(1500)
    
    
    terrain = veh.DeformableTerrain(system, soil)
    terrain.SetHeightmap(chrono.ChVectorD(0, 0, 0), 10, 10, 0.1)  
    terrain.Initialize()
    
    
    driver = veh.ChDriver()
    hmmwv.GetVehicle().SetDriver(driver)
    driver.SetThrottle(0.8)  
    driver.SetSteering(0)    
    
    
    application.Add(hmmwv.GetVisualSystem())
    application.Add(terrain.GetVisualSystem())
    
    
    application.SetTimestep(0.001)
    application.SetRealTimeFactor(1)  
    
    
    while application.GetDevice().run():
        application.BeginScene()
        application.DrawAll()
        
        
        driver.Update()
        
        
        system.DoStepDynamics(system.GetChTimeStep())
        
        application.EndScene()

if __name__ == '__main__':
    main()