from pychrono import ChSystem, ChSystemNSC, ChVector, ChBody, ChBodyAux, ChCoordsys, ChRigidBody, ChLink6D, ChTerrainModel

from pychrono.modelling import ChBeam, ChBeamAux

import pychrono.Irrlicht as IRr

import pychrono.contact as contact


def main():

    

    ch_system = ChSystemNSC()


    

    beam_length = 10.0

    beam_mass = 1.0

    beam_radius = 0.1

    beam_material = ChMaterial()

    beam_material.SetDensity(1000.0)

    beam_material.SetElasticModulus(2.1e11)

    beam_material.SetPoissonsRatio(0.3)


    beam_body = ChBodyAux(beam_mass, beam_material, ChCoordsys())

    beam_body.SetName("Beam")


    beam_aux = ChBeamAux(beam_length, beam_radius, beam_aux.I_T_S)

    beam_aux.SetName("BeamAux")


    beam = ChBeam(beam_body, beam_aux)

    beam.SetName("Beam")


    

    ch_system.AddBody(beam)


    

    ground = ChBody(1.0e10, ChMaterial())

    ground.SetName("Ground")

    ground.SetPos(ChVector(0, 0, 0))

    ch_system.AddBody(ground)


    

    hinge = ChLink6D(ChCoordsys(), beam.body(), 0, 0, 0, 0, 0, 0)

    hinge.SetName("Hinge")

    hinge.SetTiltAngles(ChVector(0, 0, 0))

    ch_system.AddLink(hinge)


    

    viewer = IRr.chrono_irrlicht(ch_system)


    

    ch_system.SetGravity(ChVector(0, -9.81, 0))


    

    terrain = ChTerrainModel()

    terrain.SetScale(1.0)

    ch_system.AddTerrain(terrain)


    

    while True:

        

        ch_system.DoStepDynamics(0.01)


        

        viewer.DrawModel(beam.body())


        

        viewer.DrawModel(ground.body())


        

        viewer.FinishDrawing()


        

        if viewer.IsClose():

            break


if __name__ == "__main__":

    main()