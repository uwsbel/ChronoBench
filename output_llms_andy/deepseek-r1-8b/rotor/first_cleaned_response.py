import pychrono as py
import numpy as np


py.init()


sim = py.Simulation()


iga_beam = py-beam.IGABeam(sim, "beam", length=1.0, cross_section="rectangle", 
                            young_modulus=21000000, density=7850, 
                            shear_modulus=None, poisson_ratio=0.0, 
                            material_name="Steel")


flywheel = py.rigid_body(sim, "flywheel", mass=10.0, 
                        moment_inertia=1/12 * np.pi * (0.2)**4, 
                        radius=0.2, 
                        inertia_shape="sphere", 
                        density=8000, 
                        material_name="Steel")


beam_end1 = py.contact_point(sim, "beam_end1", body1=iga_beam, 
                           body2=flywheel, 
                           position=[0, 0, 0], 
                           type="fixed")
beam_end2 = py.contact_point(sim, "beam_end2", body1=iga_beam, 
                           body2=None, 
                           position=[1.0, 0, 0], 
                           type="free")


motor = py.actuator(sim, "motor", 
                   type="rotational", 
                   body1=beam_end2, 
                   body2=beam_end1, 
                   torque_max=100.0, 
                   resistance=0.01, 
                   type_moment="moment")


fem_viewer = py.FemViewer(sim, "fem_viewer", 
                       type="fem", 
                       object1=iga_beam, 
                       object2=flywheel, 
                       object3=motor)


irr_light = py.IrrlichtWindow(sim, "irr_light", 
                           window_size=[800, 600], 
                           camera_position=[5, 5, 5], 
                           camera.look_at=[0, 0, 0])


beam_length = 1.0
beam_cross_section = 0.05  
beam_density = 7850
flywheel_radius = 0.2
flywheel_mass = 10.0


steel = py.material(sim, "Steel", 
                    young_modulus=21000000, 
                    shear_modulus=21000000/3, 
                    poisson_ratio=0.0, 
                    density=7850)


sim.set_gravity([0, -9.81, 0])
sim.set_time_step(1e-4)
sim.set_maximum_time_step(1.0)
sim.set_tolerances(1e-9)


def natural_frequency():
    beam_length = iga_beam.length
    beam_area = beam_cross_section * beam_length
    beam_mass = iga_beam.mass
    flywheel_mass = flywheel.mass
    total_mass = beam_mass + flywheel_mass
    total_moment_inertia = iga_beam.moment_inertia + flywheel.moment_inertia
    natural_freq = np.sqrt((3 * (beam_area) * 9.81) / total_moment_inertia)
    return natural_freq


sim.run()