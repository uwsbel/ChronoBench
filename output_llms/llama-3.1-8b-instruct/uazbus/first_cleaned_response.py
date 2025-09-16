import chrono as chrono
import chronoysics as chronoysics
import numpy as np


chrono.Initialize()


m_world = chrono.ChSystem()
m_world.SetSolverType(chrono.SolverType_HalfSemiImplicit)


m_vehicle = chrono.ChBody()
m_vehicle.SetName('UAZBUS')
m_vehicle.SetBodyType(chrono.ChBodyType_BOX)
m_vehicle.SetPos(chrono.ChVectorD(0, 0, 0))
m_vehicle.SetRot(chrono.Q_from_AngAxis(0, 0, 0, 1))
m_vehicle.SetMass(1500)
m_vehicle.SetDensity(0.8)
m_vehicle.SetFriction(0.7)
m_vehicle.SetRestitution(0.2)
m_vehicle.SetInertiaXX(1000)
m_vehicle.SetInertiaXY(0)
m_vehicle.SetInertiaXZ(0)
m_vehicle.SetInertiaYY(1000)
m_vehicle.SetInertiaYZ(0)
m_vehicle.SetInertiaZZ(1000)


m_terrain = chrono.ChBody()
m_terrain.SetName('Terrain')
m_terrain.SetBodyType(chrono.ChBodyType_BOX)
m_terrain.SetPos(chrono.ChVectorD(0, 0, 0))
m_terrain.SetRot(chrono.Q_from_AngAxis(0, 0, 0, 1))
m_terrain.SetMass(1000)
m_terrain.SetDensity(0.5)
m_terrain.SetFriction(0.5)
m_terrain.SetRestitution(0.1)
m_terrain.SetInertiaXX(500)
m_terrain.SetInertiaXY(0)
m_terrain.SetInertiaXZ(0)
m_terrain.SetInertiaYY(500)
m_terrain.SetInertiaYZ(0)
m_terrain.SetInertiaZZ(500)


m_world.Add(m_vehicle)
m_world.Add(m_terrain)


m_driver = chrono.ChDriver()
m_driver.SetName('Driver')
m_driver.SetVehicle(m_vehicle)
m_driver.SetTerrain(m_terrain)


m_visualizer = chronoysics.IrrVisualizer()
m_visualizer.SetScene(m_world)
m_visualizer.SetCamera(chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 1))
m_visualizer.SetLighting(chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 1))


while True:
    
    m_world.DoStepDynamics(chrono.SIMSTEPS, 0.01)
    m_driver.DoStepDynamics(chrono.SIMSTEPS, 0.01)
    m_visualizer.DoStepDynamics(chrono.SIMSTEPS, 0.01)
    
    
    m_world.DoStepDynamics(chrono.SIMSTEPS, 0.01)
    m_driver.DoStepDynamics(chrono.SIMSTEPS, 0.01)
    m_visualizer.DoStepDynamics(chrono.SIMSTEPS, 0.01)
    
    
    if m_world.GetTime() > 10:
        break


chrono.Destroy()