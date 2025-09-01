import pychron.core as pc

import pychron.physics as pf

import pychron.visualization as pv

import pychron.particles as pp

import pychron.emitters as pe

import pychron.colliders as pc

import pychron.gravity as pg

import pychron.visualization.irrlicht as iv




pc.initialize()




pf.add_system(pc.PhysicsSystem())




emitter = pe.ParticleEmitter(pc.PhysicsSystem())

emitter.set_emission_rate(10)  




gravity_force = pg.GravityForce(pc.PhysicsSystem(), strength=9.81)

pf.add_force(gravity_force)




irrlicht_renderer = iv.IrrlichtRenderer()




for _ in range(100):  

    particle = pp.Particle(pc.PhysicsSystem())

    particle.set_shape(pp.RandomShape())

    particle.set_position(pf.RandomPosition())

    particle.set_velocity(pf.RandomVelocity())

    particle.set_orientation(pf.RandomOrientation())

    emitter.emit(particle)




while pc.PhysicsSystem().is_running():

    

    pc.PhysicsSystem().update()


    

    irrlicht_renderer.render(emitter.get_particles())




pc.shutdown()