# HyunLab Ornithopter Project
# Written by Jaden Hernandez
import pterasoftware as ps
import numpy as np
import math
import csv
import uuid

wingChord = 100.0 / 1000.0 # chord of wing converted from millimeters to meters
foldDistance = 70.0 / 1000.0 # how far away the fold is spanwise from the robot's center in meters
totalHalfSpan = 225.0 / 1000.0 # how long one wing's span is in meters
fuselageWidth = 60 / 1000.0 # gap between the wings in meters
flightSpeed = 0.5 # translational forward speed of the flapping wing vehicle in m/s
pitchAngle = 0.0 # angle of attack of the robot in flight
AB = (15.25) / 1000 # AB linkage length
BC = (40.25) / 1000 # BC linkage length
CD = (31.035) / 1000 # CD linkage length
AD = (38.0) / 1000 # AD linkage length
phi1 = np.arccos((AD**2 + CD**2 - (BC + AB)**2) / (2 * AD * CD))
phi2 = np.arccos((AD**2 + CD**2 - (BC - AB)**2) / (2 * AD * CD))
delta = phi1 - phi2
deltaDeg = np.rad2deg(delta)
angleC2AD = np.arccos(((BC - AB)**2 + AD**2 - CD**2) / (2 * (BC - AB) * AD))
angleCAD = np.arccos(((AB + BC)**2+ AD**2 - CD**2) / (2 * (AB + BC) * AD))
theta = angleC2AD - angleCAD
thetaDeg = np.rad2deg(theta)
period = 2.0 # period of flap movement
print(np.rad2deg(phi1))
print(deltaDeg)
def crank(t):
    omega = 2 * np.pi / period
    angle = omega * t
    return angle
def coupler(t):
    angle = theta * np.sin(t)
    return angle
def rocker(t):
    #input = (AB * np.sin(crank(t)) + BC * np.sin(coupler(t))) / CD
    #input = [1 if x > 1 else x for x in input]
    #input = [-1 if x < -1 else x for x in input]
    angle = -coupler(t + np.pi / 2)
    return angle

fwv = ps.geometry.Airplane(
    name="Flapping Wing Vehicle",
    x_ref=0.0,
    y_ref=0.0,
    z_ref=0.0,

    s_ref=None,
    b_ref=None,
    c_ref=None,
    wings=[
        ps.geometry.Wing(
            name="Main Wing",
            x_le=0.0,
            y_le=0.0,
            z_le=0.0,
            symmetric=True,
            num_chordwise_panels=4,
            chordwise_spacing="uniform",
            wing_cross_sections=[
                ps.geometry.WingCrossSection(
                    x_le=0.0,
                    y_le=0.0,
                    z_le=0.0,
                    num_spanwise_panels=1,
                    spanwise_spacing="uniform",
                    chord= wingChord,
                    airfoil=ps.geometry.Airfoil(
                        name="naca0000", #flat plate
                    ),
                ),                
                ps.geometry.WingCrossSection(
                    x_le=0.0,
                    y_le=fuselageWidth / 2,
                    z_le=0.0,
                    num_spanwise_panels=4,
                    spanwise_spacing="uniform",
                    chord= wingChord,
                    airfoil=ps.geometry.Airfoil(
                        name="naca0000", #flat plate
                        coordinates=None,
                        repanel=True,
                    ),
                ),
                ps.geometry.WingCrossSection(
                    y_le= foldDistance + fuselageWidth / 2,
                    chord= wingChord,
                    num_spanwise_panels=4,
                    spanwise_spacing="uniform",
                    airfoil=ps.geometry.Airfoil(
                        name="naca0000",
                    ),
                ),                   
                ps.geometry.WingCrossSection(
                    y_le= totalHalfSpan + foldDistance + fuselageWidth / 2,
                    chord= wingChord,
                    num_spanwise_panels=4,
                    spanwise_spacing="uniform",
                    airfoil=ps.geometry.Airfoil(
                        name="naca0000",
                    ),
                ),             
            ],
        ),
    ],
)

fuselage_movement = ps.movement.WingCrossSectionMovement(
    base_wing_cross_section=fwv.wings[0].wing_cross_sections[0],
    sweeping_amplitude=0.0,
    sweeping_period=0.0,
    sweeping_spacing="sine",
    pitching_amplitude=0.0,
    pitching_period=0.0,
    pitching_spacing="sine",
    heaving_amplitude=0.0,
    heaving_period=0.0,
    heaving_spacing="sine",
)

main_wing_root_wing_cross_section_movement = ps.movement.WingCrossSectionMovement(
    base_wing_cross_section=fwv.wings[0].wing_cross_sections[1],
    sweeping_amplitude=0.0,
    sweeping_period=0.0,
    sweeping_spacing="sine",
    pitching_amplitude=0.0,
    pitching_period=0.0,
    pitching_spacing="sine",
    heaving_amplitude=0.0,
    heaving_period=0.0,
    heaving_spacing="sine",
)

main_wing_tip_wing_cross_section_movement = ps.movement.WingCrossSectionMovement(
    base_wing_cross_section=fwv.wings[0].wing_cross_sections[2],
    sweeping_amplitude=106.71 - thetaDeg,
    sweeping_period=period,
    sweeping_spacing="custom",
    custom_sweep_function= coupler,
    pitching_amplitude=0.0,
    pitching_period=0.0,
    pitching_spacing="sine",
    heaving_amplitude=0.0,
    heaving_period=0.0,
    heaving_spacing="sine",
)

main_wing_tip_wing_cross_section_movement = ps.movement.WingCrossSectionMovement(
    base_wing_cross_section=fwv.wings[0].wing_cross_sections[2],
    sweeping_amplitude=90 - thetaDeg,
    sweeping_period=period,
    sweeping_spacing="custom",
    custom_sweep_function= coupler,
    pitching_amplitude=0.0,
    pitching_period=0.0,
    pitching_spacing="sine",
    heaving_amplitude=0.0,
    heaving_period=0.0,
    heaving_spacing="sine",
)

main_wing_tip_wing_cross_section_movement2 = ps.movement.WingCrossSectionMovement(
    base_wing_cross_section=fwv.wings[0].wing_cross_sections[3],
    sweeping_amplitude=deltaDeg,
    sweeping_period=period,
    sweeping_spacing="custom",
    custom_sweep_function= rocker,
    pitching_amplitude=0.0,
    pitching_period=0.0,
    pitching_spacing="sine",
    heaving_amplitude=0.0,
    heaving_period=0.0,
    heaving_spacing="sine",
)

main_wing_movement = ps.movement.WingMovement(
    base_wing=fwv.wings[0],
    wing_cross_sections_movements=[
        fuselage_movement,
        main_wing_root_wing_cross_section_movement,
        main_wing_tip_wing_cross_section_movement,
        main_wing_tip_wing_cross_section_movement2,
    ],
    x_le_amplitude=0.0,
    x_le_period=0.0,
    x_le_spacing="sine",
    y_le_amplitude=0.0,
    y_le_period=0.0,
    y_le_spacing="sine",
    z_le_amplitude=0.0,
    z_le_period=0.0,
    z_le_spacing="sine",
)

del main_wing_root_wing_cross_section_movement
del main_wing_tip_wing_cross_section_movement

airplane_movement = ps.movement.AirplaneMovement(
    base_airplane=fwv,
    wing_movements=[main_wing_movement],
    x_ref_amplitude=0.0,
    x_ref_period=0.0,
    x_ref_spacing="sine",
    y_ref_amplitude=0.0,
    y_ref_period=0.0,
    y_ref_spacing="sine",
    z_ref_amplitude=0.0,
    z_ref_period=0.0,
    z_ref_spacing="sine",
)

del main_wing_movement

example_operating_point = ps.operating_point.OperatingPoint(
    density=1.225,
    beta=0.0,
    velocity = flightSpeed,
    alpha = pitchAngle,
    nu=15.06e-6,
)

operating_point_movement = ps.movement.OperatingPointMovement(
    base_operating_point=example_operating_point,
    velocity_amplitude=0.0,
    velocity_period=0.0,
    velocity_spacing="sine",
)

movement = ps.movement.Movement(
    airplane_movements=[airplane_movement],
    operating_point_movement=operating_point_movement,
    num_steps=None,
    delta_time=None,
)

del airplane_movement
del operating_point_movement

example_problem = ps.problems.UnsteadyProblem(
    movement=movement,
)

example_solver = ps.unsteady_ring_vortex_lattice_method.UnsteadyRingVortexLatticeMethodSolver(
    unsteady_problem=example_problem,
)

del example_problem

example_solver.run(
    logging_level="Warning",
    prescribed_wake=True,
)

ps.output.animate(
    unsteady_solver=example_solver,
    scalar_type="lift",
    show_wake_vortices=True,
    save=True,
)

scalars = ps.output.print_unsteady_results(example_solver)