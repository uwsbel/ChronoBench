import pychrono as chrono
import pychrono.irrlicht as chronoirr


chrono.SetChronoDataPath("path/to/chrono/data")  
physics = chrono.ChSystemNSC()


vis = chronoirr.ChIrrApp(physics)
vis.AddCamera(chrono.ChVectorD(5, 5, 5))  
vis.AddLight(chrono.ChVectorD(1, 1, 1))  


particle_count = 100
particle_radius = 0.1
emission_rate = 10
velocity_range = (1, 5)  
orientation_range = (0, 360)  


def create_particle():
    shape = chrono.ChBodyEasyBox(particle_radius, particle_radius, particle_radius)
    shape.SetMaterialSurface(chrono.ChMaterialSurface())
    shape.SetMass(1)  
    shape.SetPos(chrono.ChVectorD(chrono.GetRandomDouble(-5, 5), chrono.GetRandomDouble(-5, 5), chrono.GetRandomDouble(-5, 5)))
    shape.SetBodyFixed(False)
    shape.SetRot(chrono.Q_from_Euler(chrono.GetRandomDouble(0, 360), chrono.GetRandomDouble(0, 360), chrono.GetRandomDouble(0, 360)))
    shape.SetLinearVelocity(chrono.ChVectorD(chrono.GetRandomDouble(velocity_range[0], velocity_range[1]), chrono.GetRandomDouble(velocity_range[0], velocity_range[1]), chrono.GetRandomDouble(velocity_range[0], velocity_range[1])))
    return shape


for _ in range(particle_count):
    particle = create_particle()
    physics.Add(particle)





vis.SetTimestep(0.01)
while vis.Run():
    physics.DoStepDynamics(vis.GetTimestep())
    vis.Render()


vis.Close()