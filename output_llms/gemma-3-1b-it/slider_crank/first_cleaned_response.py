import pychrono
import pychrono.visuals as visuals
import pychrono.physics as physics
import pychrono.objects as objects
import math



system = objects.System()
system.add_object(objects.CrankSlider(
    x=0.0,
    y=0.0,
    z=0.0,
    rotation=0.0,  
    name="CrankSlider",
    force_applied=0.0, 
    mass=1.0,
    friction=0.0
))


floor = objects.Floor()
floor.name = "Floor"
floor.add_object(objects.CrankSlider(
    x=0.0,
    y=0.0,
    z=0.0,
    rotation=0.0,
    name="Floor",
    force_applied=0.0,
    mass=1.0,
    friction=0.0
))


crank = objects.Crank(
    name="Crank",
    x=0.0,
    y=0.0,
    z=0.0,
    rotation=0.0,
    force_applied=0.0,
    mass=1.0,
    friction=0.0
)


rod = objects.Rod(
    name="ConnectingRod",
    x=0.0,
    y=0.0,
    z=0.0,
    rotation=0.0,
    force_applied=0.0,
    mass=1.0,
    friction=0.0
)


piston = objects.Piston(
    name="Piston",
    x=0.0,
    y=0.0,
    z=0.0,
    rotation=0.0,
    force_applied=0.0,
    mass=1.0,
    friction=0.0
)



system.set_default_forces(0.0)  
system.set_default_friction(0.0) 
system.set_default_angle(0.0) 



simulation_time = 10.0  
time_step = 0.01 



visuals.create_scene(
    title="Crank-Slider Simulation",
    width=800,
    height=600,
    background="black")


floor_object = visuals.Plane(
    name="Floor",
    x=0.0,
    y=0.0,
    z=0.0,
    width=100.0,
    height=100.0,
    color="white"
)
visuals.add_object(floor_object)


crank_object = visuals.Cube(
    name="Crank",
    x=0.0,
    y=0.0,
    z=0.0,
    width=50.0,
    height=50.0,
    color="red"
)
visuals.add_object(crank_object)


rod_object = visuals.Cube(
    name="ConnectingRod",
    x=0.0,
    y=0.0,
    z=0.0,
    width=50.0,
    height=50.0,
    color="blue"
)
visuals.add_object(rod_object)


piston_object = visuals.Cube(
    name="Piston",
    x=0.0,
    y=0.0,
    z=0.0,
    width=50.0,
    height=50.0,
    color="green"
)
visuals.add_object(piston_object)



physics.set_physics_system(system)



try:
    while True:
        
        system.update()

        
        
        torque = (crank.force_applied * rod.mass) / rod.friction

        
        piston.force_applied = torque

        
        angular_velocity = crank.rotation

        
        crank.rotation = angular_velocity

        
        crank.force_applied = torque
        rod.force_applied = torque
        piston.force_applied = torque

        
        floor.x = floor.x + 0.1
        floor.y = floor.y + 0.1
        floor.z = floor.z + 0.1


        
        visuals.render(visuals.get_scene())

        
        time.sleep(time_step)

except KeyboardInterrupt:
    print("Simulation stopped.")
    break