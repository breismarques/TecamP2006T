# Vehicles.py

# Python Imports
import numpy as np
import pylab as plt
from subprocess import call
import time

# SUAVE Imports
import SUAVE
from SUAVE.Core import Data, Units
from SUAVE.Components.Energy.Networks.Battery_Propeller import Battery_Propeller
from SUAVE.Methods.Propulsion import propeller_design
from SUAVE.Input_Output.Results import  print_parasite_drag,  \
     print_compress_drag, \
     print_engine_data,   \
     print_mission_breakdown, \
     print_weight_breakdown
from SUAVE.Methods.Power.Battery.Sizing import initialize_from_energy_and_power, initialize_from_mass
from SUAVE.Input_Output.OpenVSP import vsp_write_x57
from SUAVE.Methods.Propulsion.electric_motor_sizing import size_from_kv
#from SUAVE.Input_Output.OpenVSP import vspaero

# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------

def main():
    
    #vsp file
    
    #vsp_write_x57(vehicle_setup(),'X57Opt')

    # build the vehicle, configs, and analyses
    configs, analyses = full_setup()
    
    configs.finalize()
    analyses.finalize()    
    
    # weight analysis
    weights = analyses.configs.base.weights
    breakdown = weights.evaluate()
    
    # mission analysis
    mission = analyses.missions.mission
    results = mission.evaluate()
    
    # plot results    
    plot_mission(results)
    
    #call(["/Users/Bruno/OpenVSP/build/_CPack_Packages/MacOS/ZIP/OpenVSP-3.13.3-MacOS/vsp","open","cruise.vsp3"])
    

    return

# ----------------------------------------------------------------------
#   Analysis Setup
# ----------------------------------------------------------------------

def full_setup():

    # vehicle data
    configs  = setup()

    # vehicle analyses
    configs_analyses = analyses_setup(configs)
    

    # mission analyses
    missions_analyses = missions_setup(configs_analyses,configs)

    analyses = SUAVE.Analyses.Analysis.Container()
    analyses.configs  = configs_analyses
    analyses.missions = missions_analyses
    
   

    return configs, analyses

# ----------------------------------------------------------------------
#   Define the Vehicle Analyses
# ----------------------------------------------------------------------

def analyses_setup(configs):

    analyses = SUAVE.Analyses.Analysis.Container()

    # build a base analysis for each config
    for tag,config in configs.items():
        analysis = base_analysis(config)
        analyses[tag] = analysis

    return analyses

def base_analysis(vehicle):

    # ------------------------------------------------------------------
    #   Initialize the Analyses
    # ------------------------------------------------------------------     
    analyses = SUAVE.Analyses.Vehicle()
    
    # ------------------------------------------------------------------
    #  Basic Geometry Relations
    sizing = SUAVE.Analyses.Sizing.Sizing()
    sizing.features.vehicle = vehicle
    analyses.append(sizing)
    
    # ------------------------------------------------------------------
    #  Weights
    weights = SUAVE.Analyses.Weights.Weights_Tube_Wing()
    weights.settings.empty_weight_method = \
        SUAVE.Methods.Weights.Correlations.Human_Powered.empty
    weights.vehicle = vehicle
    analyses.append(weights)
    
    # ------------------------------------------------------------------
    #  Energy
    energy = SUAVE.Analyses.Energy.Energy()
    energy.network = vehicle.propulsors #what is called throughout the mission (at every time step))
    analyses.append(energy)
    
    # ------------------------------------------------------------------
    #  Aerodynamics Analysis
    aerodynamics = SUAVE.Analyses.Aerodynamics.Open_VSP_Analysis_No_Surrogates()
    aerodynamics.geometry = vehicle
    aerodynamics.settings.drag_coefficient_increment = 0.0000
    #aerodynamics.settings.maximum_lift_coefficient = np.inf
    analyses.append(aerodynamics)
    
    # ------------------------------------------------------------------
    #  Planet Analysis
    planet = SUAVE.Analyses.Planets.Planet()
    analyses.append(planet)
    
    # ------------------------------------------------------------------
    #  Atmosphere Analysis
    atmosphere = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    atmosphere.features.planet = planet.features
    analyses.append(atmosphere)
    
    # done!
    return analyses    

def setup():
    
    base_vehicle = vehicle_setup()
    configs = configs_setup(base_vehicle)
    
    return configs

# ----------------------------------------------------------------------
#   Define the Vehicle
# ----------------------------------------------------------------------

def vehicle_setup():
    
    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------    
    
    vehicle = SUAVE.Vehicle()
    vehicle.tag = 'Tecnam_P2006TElectric'    
    
    # ------------------------------------------------------------------
    #   Vehicle-level Properties
    # ------------------------------------------------------------------    

    # mass properties
    vehicle.mass_properties.max_takeoff               = 1400* Units.kilogram 
    vehicle.mass_properties.takeoff                   = 1400 * Units.kilogram   
    vehicle.mass_properties.operating_empty           = 1000 * Units.kilogram 
    vehicle.mass_properties.max_zero_fuel             = 1400 * Units.kilogram 
    vehicle.mass_properties.cargo                     = 80  * Units.kilogram   
    
    # envelope properties
    vehicle.envelope.ultimate_load = 5.7
    vehicle.envelope.limit_load    = 3.8

    # basic parameters
    vehicle.reference_area         = 64.4 * Units['meters**2']  
    vehicle.passengers             = 4
    vehicle.systems.control        = "fully powered" 
    vehicle.systems.accessories    = "medium range"

    # ------------------------------------------------------------------        
    #  Landing Gear
    # ------------------------------------------------------------------        
    # used for noise calculations
    #landing_gear = SUAVE.Components.Landing_Gear.Landing_Gear()
    #landing_gear.tag = "main_landing_gear"
    
    #landing_gear.main_tire_diameter = 0.423 * Units.m
    #landing_gear.nose_tire_diameter = 0.3625 * Units.m
    #landing_gear.main_strut_length  = 0.4833 * Units.m
    #landing_gear.nose_strut_length  = 0.3625 * Units.m
    #landing_gear.main_units  = 2    #number of main landing gear units
    #landing_gear.nose_units  = 1    #number of nose landing gear
    #landing_gear.main_wheels = 1    #number of wheels on the main landing gear
    #landing_gear.nose_wheels = 1    #number of wheels on the nose landing gear      
    #vehicle.landing_gear = landing_gear
    
    # ------------------------------------------------------------------
    #  Fuselage
    # ------------------------------------------------------------------
    
    fuselage = SUAVE.Components.Fuselages.Fuselage()    
    fuselage.tag = 'fuselage'
    aux = time.time()
    fuselage.time = aux
    
    fuselage.number_coach_seats    = vehicle.passengers
    fuselage.seats_abreast         = 2
    fuselage.seat_pitch            = 0.995     * Units.meter
    fuselage.fineness.nose         = 1.27
    fuselage.fineness.tail         = 1  #3.31
    fuselage.lengths.nose          = 1.16  * Units.meter
    fuselage.lengths.tail          = 4.637 * Units.meter
    fuselage.lengths.cabin         = 2.653 * Units.meter
    fuselage.lengths.total         = 8.45 * Units.meter
    fuselage.lengths.fore_space    =  0.0   * Units.meter
    fuselage.lengths.aft_space     =  0.0   * Units.meter
    fuselage.width                 = 1.1  * Units.meter   #1.22
    fuselage.heights.maximum       = 1.41  * Units.meter
    fuselage.effective_diameter    =  2 * Units.meter
    fuselage.areas.side_projected  = 7.46  * Units['meters**2'] 
    fuselage.areas.wetted          = 25.0  * Units['meters**2'] 
    fuselage.areas.front_projected = 1.54 * Units['meters**2'] 
    fuselage.differential_pressure = 10**5 * Units.pascal # Maximum differential pressure
    
    fuselage.heights.at_quarter_length          = 1.077 * Units.meter
    fuselage.heights.at_three_quarters_length   =  0.5 * Units.meter   #0.621 * Units.meter
    fuselage.heights.at_wing_root_quarter_chord = 1.41  * Units.meter
    
    ## OpenVSP Design
    
    fuselage.OpenVSP_values = Data() # VSP uses degrees directly
    
    #MidFuselage1 Section
    
    fuselage.OpenVSP_values.midfus1 = Data()
    fuselage.OpenVSP_values.midfus1.z_pos=0.03
    
    #MidFuselage2 Section
    
    fuselage.OpenVSP_values.midfus2 = Data()
    fuselage.OpenVSP_values.midfus2.z_pos=0.06
    
    #MidFuselage3 Section
    
    fuselage.OpenVSP_values.midfus3 = Data()
    fuselage.OpenVSP_values.midfus3.z_pos=0.04
    
    #Tail Section
    
    fuselage.OpenVSP_values.tail = Data() 
    fuselage.OpenVSP_values.tail.bottom = Data()
    fuselage.OpenVSP_values.tail.z_pos = 0.039
    fuselage.OpenVSP_values.tail.bottom.angle = -20.0
    fuselage.OpenVSP_values.tail.bottom.strength = 1

   
    
    # add to vehicle
    vehicle.append_component(fuselage)


    # ------------------------------------------------------------------        
    #   Main Wing
    # ------------------------------------------------------------------        
    
    wing = SUAVE.Components.Wings.Main_Wing()
    wing.tag = 'main_wing'
    
    wing.thickness_to_chord      = 0.16093948672520503
    wing.taper                   = 0.8890605105967725
    wing.spans.projected         = 9.356242633421443 * Units.meter
    wing.chords.root             = 1.9687442509506163 * Units.meter
    wing.chords.tip              = wing.chords.root*wing.taper
    wing.chords.mean_aerodynamic = (wing.chords.root*(2.0/3.0)*((1.0+wing.taper+wing.taper**2.0)/(1.0+wing.taper)))
    wing.areas.reference         = (wing.chords.root+wing.chords.tip)*wing.spans.projected/2
    basearea                     = wing.areas.reference
    wing.areas.wetted            = 2.0 * wing.areas.reference
    wing.areas.exposed           = wing.areas.wetted
    wing.areas.affected          = wing.areas.wetted
    wing.twists.root             = 0. * Units.degrees
    wing.twists.tip              = 0. * Units.degrees
    wing.dihedral= 1. * Units.degrees
    wing.origin                  = [2.986,0,1.077] # meters
    wing.sweeps.leading_edge     = 1.9 * Units.deg
    wing.aspect_ratio            = (wing.spans.projected*wing.spans.projected)/wing.areas.reference
    wing.span_efficiency         = 0.95
    wing.vertical                = False
    wing.symmetric               = True
    wing.high_lift               = True
    wing.dynamic_pressure_ratio  = 1.0

    
     
    ## Wing Segments
    
    # Root Segment
    
    #segment = SUAVE.Components.Wings.Segment()
    
    #segment.tag                   = 'root'
    #segment.percent_span_location = 0.0
    #segment.twist                 = 0. * Units.deg
    #segment.root_chord_percent    = 1.
    #segment.dihedral_outboard     = 1. * Units.degrees
    #segment.sweeps.quarter_chord  = 0. * Units.degrees
    #segment.thickness_to_chord    = 0.15
    
    #airfoil = SUAVE.Components.Wings.Airfoils.Airfoil()
    #airfoil.coordinate_file       = '/Users/Bruno/Documents/Delft/Courses/2016-2017/Thesis/Code/Airfoils/naca642415.dat'
    
    #segment.append_airfoil(airfoil)
    #wing.Segments.append(segment)
    
    
    # Tip Segment
    
    #segment = SUAVE.Components.Wings.Segment()
    
    #segment.tag                   = 'tip'
    #segment.percent_span_location = 1.0
    #segment.twist                 = 0. * Units.deg
    #segment.root_chord_percent    = 1.
    #segment.dihedral_outboard     = 1. * Units.degrees
    #segment.sweeps.quarter_chord  = 0. * Units.degrees
    #segment.thickness_to_chord    = 0.15
    
    #airfoil = SUAVE.Components.Wings.Airfoils.Airfoil()
    #airfoil.coordinate_file       = '/Users/Bruno/Documents/Delft/Courses/2016-2017/Thesis/Code/Airfoils/naca642415.dat'
    
    #segment.append_airfoil(airfoil)  
    #wing.Segments.append(segment)
    
    # ------------------------------------------------------------------
    #   Flaps
    # ------------------------------------------------------------------
    
    wing.flaps.chord      =  wing.chords.root*0.15   
    wing.flaps.span_start =  0.3 * wing.spans.projected
    wing.flaps.span_end   =  0.8 * wing.spans.projected
    wing.flaps.area       = wing.flaps.chord * (wing.flaps.span_end-wing.flaps.span_start)
    wing.flaps.type       = 'single_slotted'

    # add to vehicle
    vehicle.append_component(wing)

    # ------------------------------------------------------------------        
    #  Horizontal Stabilizer
    # ------------------------------------------------------------------        
    
    wing = SUAVE.Components.Wings.Wing()
    wing.tag = 'horizontal_stabilizer'
    
    
    wing.aspect_ratio            = 4.193
    wing.areas.reference         = 0.145 * basearea     
    wing.sweeps.quarter_chord    = 0.0 * Units.deg
    wing.thickness_to_chord      = 0.12
    wing.taper                   = 1.0
    wing.span_efficiency         = 0.97
    wing.spans.projected         = np.sqrt(wing.aspect_ratio*wing.areas.reference)
    wing.chords.root             = wing.areas.reference/wing.spans.projected
    wing.chords.tip              = wing.chords.root
    wing.chords.mean_aerodynamic = (wing.chords.root*(2.0/3.0)*((1.0+wing.taper+wing.taper**2.0)/(1.0+wing.taper)))
    wing.areas.wetted            = 2.0 * wing.areas.reference
    wing.areas.exposed           = wing.areas.wetted
    wing.areas.affected          = wing.areas.wetted
    wing.twists.root             = 0.0 * Units.degrees
    wing.twists.tip              = 0.0 * Units.degrees  
    wing.origin                  = [7.789,0.0,0.3314] # meters
    wing.vertical                = False 
    wing.symmetric               = True
    wing.dynamic_pressure_ratio  = 0.9  
    
    # add to vehicle
    vehicle.append_component(wing)
    
    # ------------------------------------------------------------------
    #   Vertical Stabilizer
    # ------------------------------------------------------------------
    
    wing = SUAVE.Components.Wings.Wing()
    wing.tag = 'vertical_stabilizer'    
    
    
    wing.areas.reference         = 0.099 * basearea
    wing.areas.wetted            = 2.0 * wing.areas.reference
    wing.areas.exposed           = wing.areas.wetted
    wing.areas.affected          = wing.areas.wetted
    wing.aspect_ratio            = 1.407
    wing.sweeps.quarter_chord    = 38.75 * Units.deg
    wing.thickness_to_chord      = 0.12
    wing.taper                   = 0.4142
    wing.span_efficiency         = 0.97
    wing.spans.projected         = np.sqrt(wing.aspect_ratio*wing.areas.reference)
    wing.chords.root             = (2.0*wing.areas.reference)/(wing.spans.projected*(1+wing.taper))
    wing.chords.tip              = wing.chords.root*wing.taper
    wing.chords.mean_aerodynamic = (wing.chords.root*(2.0/3.0)*((1.0+wing.taper+wing.taper**2.0)/(1.0+wing.taper)))  
    wing.twists.root             = 0.0 * Units.degrees
    wing.twists.tip              = 0.0 * Units.degrees  
    wing.origin                  = [7.25,0,0.497] # meters
    wing.vertical                = True 
    wing.symmetric               = False
    wing.t_tail                  = False
    wing.dynamic_pressure_ratio  = 1.0
        
    # add to vehicle
    vehicle.append_component(wing)

    # ------------------------------------------------------------------
    #  Propellers Powered By Batteries
    # ------------------------------------------------------------------    
    
    # build network
    net = SUAVE.Components.Energy.Networks.Lift_Forward_Propulsor()
    
    net.nacelle_diameter_lift     = 0.08 * Units.meters
    net.nacelle_diameter_forward  = 0.1732 * Units.meters
    net.engine_length_lift        = 0.47244 * Units.meters
    net.engine_length_forward     = 1.2 * Units.meters
    net.number_of_engines_lift    = 12
    net.number_of_engines_forward = 2
    net.thrust_angle_lift         = 0.0 * Units.degrees
    net.thrust_angle_forward      = 0.0 * Units.degrees
    net.voltage           = 470.0*0.4115008413707807 * Units['volt']  #461.
    net.areas_forward             = Data()
    net.areas_forward.wetted      = 1.1*np.pi*net.nacelle_diameter_forward*net.engine_length_forward
    net.areas_lift             = Data()
    net.areas_lift.wetted      = 1.1*np.pi*net.nacelle_diameter_forward*net.engine_length_lift
    net.number_of_engines         = 1
    net.nacelle_diameter          = net.nacelle_diameter_forward 
    net.areas                    = Data()
    net.areas.wetted              =  net.areas_lift.wetted
    net.engine_length            = 1.
    
    # Component 1 - Tip ESC 
    esc = SUAVE.Components.Energy.Distributors.Electronic_Speed_Controller()
    esc.efficiency = 0.95 # Gundlach for brushless motors
    net.esc_forward        = esc
    
    # Component 1 - High Lift ESC 
    esc = SUAVE.Components.Energy.Distributors.Electronic_Speed_Controller()
    esc.efficiency = 0.95 # Gundlach for brushless motors
    net.esc_lift        = esc
    
    # Component 2 - Tip Propeller
    
    # Design the Propeller
    #prop_attributes = Data()
    
    
    #prop_attributes                     = propeller_design(prop_attributes)
    
    prop_forward = SUAVE.Components.Energy.Converters.Propeller()
    prop_forward.number_blades       = 3.0
    prop_forward.propulsive_efficiency = 0.85
    prop_forward.freestream_velocity = 50.0 * Units['m/s'] # freestream m/s
    prop_forward.angular_velocity    = 27500. * Units['rpm'] # For 20x10 prop
    prop_forward.tip_radius          = 0.5 * Units.meter
    prop_forward.hub_radius          = 0.085 * Units.meter
    prop_forward.design_Cl           = 0.8
    prop_forward.design_altitude     = 14.0 * Units.km
    prop_forward.design_thrust       = 0.0 * Units.newton
    prop_forward.design_power        = 60000. * Units.watts 
    
    prop_forward  = propeller_design(prop_forward)
    net.propeller_forward = prop_forward
    
    # Component 2 - High Lift Propeller
    
    # Design the Propeller
    #prop_attributes = Data()  
    
    prop_lift = SUAVE.Components.Energy.Converters.Propeller()
    prop_lift.number_blades       = 5.0
    prop_lift.propulsive_efficiency = 0.85
    prop_lift.freestream_velocity = 1. * Units['m/s'] # freestream m/s
    prop_lift.angular_velocity    = 2750 * Units['rpm'] # For 20x10 prop
    prop_lift.tip_radius          = 0.26 * Units.meter  #0.2880360
    prop_lift.hub_radius          = 0.07772400 * Units.meter
    prop_lift.design_Cl           = 0.8
    prop_lift.design_altitude     = 0.0 * Units.meter
    prop_lift.design_thrust       = 0.0 * Units.newton
    prop_lift.design_power        = 10500. * Units.watts 
    prop_lift                     = propeller_design(prop_lift)

    net.propeller_lift = prop_lift
    
    
    # Component 3 - Tip Motor
    
    motor = SUAVE.Components.Energy.Converters.Motor_Lo_Fid()
    #motor.resistance           = 1.
    #motor.no_load_current      = 7.  * Units.ampere
    #motor.speed_constant       = 11.9999 * Units['rpm/volt'] # RPM/volt converted to (rad/s)/volt    
    #motor.propeller_radius     = prop_forward.tip_radius
    #motor.propeller_Cp         = prop_forward.Cp[0]
    #motor.gear_ratio           = 12. # Gear ratio
    #motor.gearbox_efficiency   = .98 # Gear box efficiency
    #motor.expected_current     = 160. # Expected current
    #motor.mass_properties.mass = 9.0  * Units.kg
    #net.motor_forward          = motor
    
    motor.speed_constant       = 181.0*1.005689280808985 * Units['rpm/volt'] # RPM/volt is standard
    motor                      = size_from_kv(motor, motor.speed_constant)    
    motor.gear_ratio           = 1. # Gear ratio, no gearbox
    motor.gearbox_efficiency   = .98 # Gear box efficiency, no gearbox
    motor.motor_efficiency     = 0.825;
    net.motor_forward                  = motor
    
    # Component 3 - High Lift Motor
    
    motor = SUAVE.Components.Energy.Converters.Motor_Lo_Fid()
    #motor.resistance           = 0.008
    #motor.no_load_current      = 4.5  * Units.ampere
    #motor.speed_constant       = 5800. * Units['rpm/volt'] # RPM/volt converted to (rad/s)/volt
    #motor.propeller_radius     = prop_lift.tip_radius
    #motor.propeller_Cp         = prop_lift.Cp[0]
    #motor.gear_ratio           = 12. # Gear ratio
    #motor.gearbox_efficiency   = .98 # Gear box efficiency
    #motor.expected_current     = 25. # Expected current
    #motor.mass_properties.mass = 6.0  * Units.kg
    #net.motor_lift             = motor
    
    
    motor.speed_constant       = 91.0*1.5867908746904014 * Units['rpm/volt'] # RPM/volt is standard
    motor                      = size_from_kv(motor, motor.speed_constant)    
    motor.gear_ratio           = 1. # Gear ratio, no gearbox
    motor.gearbox_efficiency   = .98 # Gear box efficiency, no gearbox
    motor.motor_efficiency     = 0.825;
    net.motor_lift                  = motor
    
    # Component 4 - the Payload
    payload = SUAVE.Components.Energy.Peripherals.Payload()
    payload.power_draw           = 50.1*0.447815767937655 * Units.watts 
    payload.mass_properties.mass = 5.0 * Units.kg
    net.payload                  = payload
    
    # Component 5 - the Avionics
    avionics = SUAVE.Components.Energy.Peripherals.Avionics()
    avionics.power_draw = 50.1*0.10184141226548901 * Units.watts
    net.avionics        = avionics      

    # Component 6 - the Battery
    bat = SUAVE.Components.Energy.Storages.Batteries.Constant_Mass.Lithium_Ion()
    bat.mass_properties.mass = 386.0 * Units.kg
    bat.specific_energy      = 6.1*30.95784647439264 * Units.Wh/Units.kg  #192.84
    bat.specific_power       = 0.833*1.0 * Units.kW/Units.kg  #0.837
    bat.resistance           = 0.0151*3321.49716638036 * Units['ohm']
    bat.max_voltage          = 60.1 * Units['volt']   #10000.
    initialize_from_mass(bat,bat.mass_properties.mass)
    net.battery              = bat
   
    
    # ------------------------------------------------------------------
    #   Vehicle Definition Complete
    # ------------------------------------------------------------------
    
    # add the energy network to the vehicle
    vehicle.append_component(net)
    
    #now add weights objects
    vehicle.landing_gear       = SUAVE.Components.Landing_Gear.Landing_Gear()
    vehicle.control_systems    = SUAVE.Components.Physical_Component()
    vehicle.electrical_systems = SUAVE.Components.Physical_Component()
    vehicle.avionics           = SUAVE.Components.Energy.Peripherals.Avionics()
    vehicle.passenger_weights  = SUAVE.Components.Physical_Component()
    #vehicle.furnishings        = SUAVE.Components.Physical_Component()
    #vehicle.apu                = SUAVE.Components.Physical_Component()
    #vehicle.hydraulics         = SUAVE.Components.Physical_Component()
    vehicle.optionals          = SUAVE.Components.Physical_Component()
    
    vehicle.wings['vertical_stabilizer'].rudder = SUAVE.Components.Physical_Component()
    
    #print vehicle

    return vehicle

# ----------------------------------------------------------------------
#   Define the Configurations
# ---------------------------------------------------------------------

def configs_setup(vehicle):
    
    # ------------------------------------------------------------------
    #   Initialize Configurations
    # ------------------------------------------------------------------
    configs = SUAVE.Components.Configs.Config.Container()

    base_config = SUAVE.Components.Configs.Config(vehicle)
    base_config.tag = 'base'
    configs.append(base_config)
    
    
    # ------------------------------------------------------------------
    #   Takeoff Configuration
    # ------------------------------------------------------------------

    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'takeoff'

    config.wings['main_wing'].flaps.angle = 20. * Units.deg
    config.wings['main_wing'].slats.angle = 25. * Units.deg

    #config.V2_VS_ratio = 1.21
    #config.maximum_lift_coefficient = 2.

    configs.append(config)
    
    

    # ------------------------------------------------------------------
    #   Cruise Configuration
    # ------------------------------------------------------------------
    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'cruise'
    config.propulsors.propulsor.number_of_engines_lift = 0
    configs.append(config)
    
    
    # ------------------------------------------------------------------
    #   Landing Configuration
    # ------------------------------------------------------------------

    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'landing'

    config.wings['main_wing'].flaps_angle = 30. * Units.deg
    config.wings['main_wing'].slats_angle = 25. * Units.deg

    config.Vref_VS_ratio = 1.23
    config.maximum_lift_coefficient = 2.

    configs.append(config)

    return configs

def simple_sizing(configs):

    base = configs.base
    base.pull_base()

    # zero fuel weight
    base.mass_properties.max_zero_fuel = 0.9 * base.mass_properties.max_takeoff 

    # wing areas
    for wing in base.wings:
        wing.areas.wetted   = 2.0 * wing.areas.reference
        wing.areas.exposed  = 0.8 * wing.areas.wetted
        wing.areas.affected = 0.6 * wing.areas.wetted

    # diff the new data
    base.store_diff()

    # ------------------------------------------------------------------
    #   Landing Configuration
    # ------------------------------------------------------------------
    landing = configs.landing

    # make sure base data is current
    landing.pull_base()

    # landing weight
    landing.mass_properties.landing = 0.85 * base.mass_properties.takeoff

    # diff the new data
    landing.store_diff()
    
    return

def missions_setup(analyses,vehicle):

    # the mission container
    missions = SUAVE.Analyses.Mission.Mission.Container()   
    missions.mission = mission_setup(analyses,vehicle)

    return missions
    

# ----------------------------------------------------------------------
#   Define the Mission
# ----------------------------------------------------------------------

def mission_setup(analyses,vehicle):

    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission = SUAVE.Analyses.Mission.Sequential_Segments()
    mission.tag = 'The Test Mission'

    mission.atmosphere  = SUAVE.Attributes.Atmospheres.Earth.US_Standard_1976()
    mission.planet      = SUAVE.Attributes.Planets.Earth()
    
    #airport
    airport = SUAVE.Attributes.Airports.Airport()
    airport.altitude   =  0.0  * Units.ft
    airport.delta_isa  =  0.0
    airport.atmosphere =  SUAVE.Analyses.Atmospheric.US_Standard_1976()

    mission.airport = airport
    
    
    # ------------------------------------------------------------------
    #   First Climb Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------
    
    # unpack Segments module
    Segments = SUAVE.Analyses.Mission.Segments
    
    # all motors segment
    no_lift_segment = Segments.Segment()   
    ones_row     = no_lift_segment.state.ones_row
    
    
    no_lift_segment.state.unknowns.lift_throttle                    = 0.8   * ones_row(1)
    no_lift_segment.state.unknowns.propeller_power_coefficient_lift = vehicle.base.propulsors.propulsor.propeller_lift.Cp[0]  * ones_row(1)
    no_lift_segment.state.unknowns.propeller_power_coefficient  = vehicle.base.propulsors.propulsor.propeller_forward.Cp[0]  * ones_row(1)
    no_lift_segment.state.unknowns.battery_voltage_under_load   = vehicle.base.propulsors.propulsor.battery.max_voltage * ones_row(1)
    
    no_lift_segment.process.iterate.unknowns.network  = vehicle.base.propulsors.propulsor.unpack_unknowns
    no_lift_segment.process.iterate.residuals.network = vehicle.base.propulsors.propulsor.residuals   
    no_lift_segment.process.iterate.initials.initialize_battery = SUAVE.Methods.Missions.Segments.Common.Energy.initialize_battery
    no_lift_segment.state.residuals.network           = 0. * ones_row(4)

    segment = Segments.Climb.Constant_Speed_Constant_Rate_VSP(no_lift_segment)
    segment.tag = "climb_1"

    segment.analyses.extend(analyses.takeoff)

    
    segment.state.numerics.number_control_points = 8
    segment.altitude_start = 0.0   * Units.km
    segment.altitude_end   = 3.0*0.33333474060829965   * Units.km
    segment.air_speed      = 125.0*1.7963130438201214 * Units['m/s']
    segment.climb_rate     = 6.0*0.5000000362103203   * Units['m/s']
    segment.battery_energy = vehicle.base.propulsors.propulsor.battery.max_energy

    # add to misison
    mission.append_segment(segment)
    
    # ------------------------------------------------------------------
    #   Second Climb Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------
    
    # unpack Segments module
    Segments = SUAVE.Analyses.Mission.Segments

    segment_2 = Segments.Climb.Constant_Speed_Constant_Rate_VSP(no_lift_segment)
    segment_2.tag = "climb_2"

    segment_2.analyses.extend(analyses.base)

    
    segment_2.state.numerics.number_control_points = 8
    segment_2.altitude_end   = 8.0*1.875   * Units.km
    segment_2.air_speed      = 190.0*1.3157894736842106 * Units['m/s']
    segment_2.climb_rate     = 3.0*0.6666666666666666   * Units['m/s']

    # add to misison
    mission.append_segment(segment_2)
    
    
    # ------------------------------------------------------------------    
    #   Cruise Segment: constant speed, constant altitude
    # ------------------------------------------------------------------  
    
    # unpack Segments module
    #Segments = SUAVE.Analyses.Mission.Segments
    
    
    # no lifting motors segment
    #no_lift_segment = Segments.Segment()   
    #ones_row     = no_lift_segment.state.ones_row
    
    
    no_lift_segment.state.unknowns.lift_throttle                    = 0.8   * ones_row(1)
    no_lift_segment.state.unknowns.propeller_power_coefficient_lift = 0.01 * ones_row(1)
    #no_lift_segment.state.unknowns.propeller_power_coefficient  = vehicle.base.propulsors.propulsor.propeller_forward.Cp[0]  * ones_row(1)
    #no_lift_segment.state.unknowns.battery_voltage_under_load   = vehicle.base.propulsors.propulsor.battery.max_voltage * ones_row(1)
    no_lift_segment.state.unknowns.__delitem__('propeller_power_coefficient_lift')
    no_lift_segment.state.unknowns.__delitem__('lift_throttle')
    #no_lift_segment.state.unknowns.throttle=0.8   * ones_row(1)
    
    no_lift_segment.process.iterate.unknowns.network  = vehicle.base.propulsors.propulsor.unpack_unknowns_no_lift
    no_lift_segment.process.iterate.residuals.network = vehicle.base.propulsors.propulsor.residuals_no_lift    
    #no_lift_segment.process.iterate.initials.initialize_battery = SUAVE.Methods.Missions.Segments.Common.Energy.initialize_battery
    no_lift_segment.state.residuals.network           = 0. * ones_row(2)
    
    segment = SUAVE.Analyses.Mission.Segments.Cruise.Constant_Speed_Constant_Altitude_VSP(no_lift_segment)
    segment.tag = "cruise"
    
    # connect vehicle configuration
    segment.analyses.extend(analyses.cruise)
    
    # segment attributes     
    segment.state.numerics.number_control_points = 8
    segment.altitude       = segment_2.altitude_end
    segment.air_speed  = 200.0*1.0074337561948112 * Units['m/s']
    segment.distance       = 150.0*3.3333333333333335 * Units.nautical_miles
    #segment.battery_energy = vehicle.base.propulsors.propulsor.battery.max_energy
    
    mission.append_segment(segment)    
    
    # ------------------------------------------------------------------
    #   First Descent Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------
    
    # unpack Segments module
    #Segments = SUAVE.Analyses.Mission.Segments
    
    # all motors segment
    #no_lift_segment = Segments.Segment()   
    #ones_row     = no_lift_segment.state.ones_row
    
    
    no_lift_segment.state.unknowns.lift_throttle                    = 0.8   * ones_row(1)
    no_lift_segment.state.unknowns.propeller_power_coefficient_lift = vehicle.base.propulsors.propulsor.propeller_lift.Cp[0]  * ones_row(1)
    no_lift_segment.state.unknowns.propeller_power_coefficient  = vehicle.base.propulsors.propulsor.propeller_forward.Cp[0]  * ones_row(1)
    no_lift_segment.state.unknowns.battery_voltage_under_load   = vehicle.base.propulsors.propulsor.battery.max_voltage * ones_row(1)
    
    no_lift_segment.process.iterate.unknowns.network  = vehicle.base.propulsors.propulsor.unpack_unknowns
    no_lift_segment.process.iterate.residuals.network = vehicle.base.propulsors.propulsor.residuals   
    #no_lift_segment.process.iterate.initials.initialize_battery = SUAVE.Methods.Missions.Segments.Common.Energy.initialize_battery
    no_lift_segment.state.residuals.network           = 0. * ones_row(4)

    segment = Segments.Descent.Constant_Speed_Constant_Rate_VSP(no_lift_segment)
    segment.tag = "descent_1"

    segment.analyses.extend(analyses.base)

    
    segment.state.numerics.number_control_points = 8
    segment.altitude_end = 3.0*1.8833448004538382   * Units.km
    segment.air_speed    = 180.0*1.3888888888888888 * Units['m/s']
    segment.descent_rate = 4.5*0.6666666666666666   * Units['m/s']
    #segment.battery_energy = vehicle.base.propulsors.propulsor.battery.max_energy

    # add to misison
    mission.append_segment(segment)
    
    # ------------------------------------------------------------------
    #   Second Descent Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------
    
    # unpack Segments module
    Segments = SUAVE.Analyses.Mission.Segments

    segment = Segments.Descent.Constant_Speed_Constant_Rate_VSP(no_lift_segment)
    segment.tag = "descent_2"

    segment.analyses.extend(analyses.landing)

    
    segment.state.numerics.number_control_points = 8
    segment.altitude_end   = 0.0   * Units.km
    segment.air_speed      = 145.0*1.2228947538694759 * Units['m/s']
    segment.descent_rate     = 3.0*0.6666668613174899  * Units['m/s']

    # add to misison
    mission.append_segment(segment)
    

    # ------------------------------------------------------------------    
    #   Mission definition complete    
    # ------------------------------------------------------------------
    
    return mission  

# ----------------------------------------------------------------------
#   Plot Mission
# ----------------------------------------------------------------------

def plot_mission(results):

    # ------------------------------------------------------------------    
    #   Throttle
    # ------------------------------------------------------------------
    plt.figure("Throttle History")
    axes = plt.gca()
    i=0
    
    
    file=open('PlottedResults.txt', 'ab')
    
    time_full=[]
    throttle_cruise_full=[]
    throttle_HL_full=[]
    for segment in results.segments.values():
        time = segment.conditions.frames.inertial.time[:,0] / Units.min
        time_full.extend(time)
        eta  = segment.conditions.propulsion.throttle[:,0]
        throttle_cruise_full.extend(eta)
        eta_lift = segment.conditions.propulsion.lift_throttle[:,0]
        throttle_HL_full.extend(eta_lift)
        
        axes.plot(time, eta, 'ko-', label='Cruise Throttle' if i == 0 else "")
        axes.plot(time, eta_lift, 'bo-', label='HL Throttle' if i == 0 else "")
        i=i+1
        
    file.write('Time (min) = '+str(time_full)+'\n')
    file.write('\n')
    file.write('Throttle Cruise = '+str(throttle_cruise_full)+'\n')
    file.write('Throttle Lift = '+str(throttle_HL_full)+'\n')
    file.write('\n')
    axes.set_xlabel('Time (mins)')
    axes.set_ylabel('Throttle')
    axes.get_yaxis().get_major_formatter().set_scientific(False)
    axes.get_yaxis().get_major_formatter().set_useOffset(False)
    plt.legend(loc='best')
    plt.ylim((0,1))
    axes.grid(True)         

    # ------------------------------------------------------------------    
    #   Altitude
    # ------------------------------------------------------------------
    plt.figure("Altitude")
    axes = plt.gca()
    altitude_full  =[]  
    for segment in results.segments.values():     
        time     = segment.conditions.frames.inertial.time[:,0] / Units.min
        altitude = segment.conditions.freestream.altitude[:,0] / Units.km
        axes.plot(time, altitude, 'bo-')
        altitude_full.extend(altitude)
        
    file.write('Altitude (Km) = '+str(altitude_full)+'\n')
    file.write('\n')
    axes.set_xlabel('Time (mins)')
    axes.set_ylabel('Altitude (km)')
    axes.grid(True)    
    
    # ------------------------------------------------------------------    
    #   Aerodynamics
    # ------------------------------------------------------------------
    fig = plt.figure("Aerodynamic Forces")
    lift_full=[]
    drag_full=[]
    thrust_full=[]
    
    for segment in results.segments.values():
        
        time   = segment.conditions.frames.inertial.time[:,0] / Units.min
        Lift   = -segment.conditions.frames.wind.lift_force_vector[:,2]
        lift_full.extend(Lift)
        Drag   = -segment.conditions.frames.wind.drag_force_vector[:,0]
        drag_full.extend(Drag)
        Thrust = segment.conditions.frames.body.thrust_force_vector[:,0]
        thrust_full.extend(Thrust)

        axes = fig.add_subplot(3,1,1)
        axes.plot( time , Lift , 'bo-' )
        axes.set_xlabel('Time (min)')
        axes.set_ylabel('Lift (N)')
        axes.get_yaxis().get_major_formatter().set_scientific(False)
        axes.get_yaxis().get_major_formatter().set_useOffset(False)          
        axes.grid(True)
        
        axes = fig.add_subplot(3,1,2)
        axes.plot( time , Drag , 'bo-' )
        axes.set_xlabel('Time (min)')
        axes.set_ylabel('Drag (N)')
        axes.get_yaxis().get_major_formatter().set_scientific(False)
        axes.get_yaxis().get_major_formatter().set_useOffset(False)          
        axes.grid(True)
        
        axes = fig.add_subplot(3,1,3)
        axes.plot( time , Thrust , 'bo-' )
        axes.set_xlabel('Time (min)')
        axes.set_ylabel('Thrust (N)')
        axes.get_yaxis().get_major_formatter().set_scientific(False)
        axes.get_yaxis().get_major_formatter().set_useOffset(False)  
        plt.ylim((-10,2500))
        axes.grid(True)
        
    file.write('Lift (N) = '+str(lift_full)+'\n')
    file.write('Drag (N) = '+str(drag_full)+'\n')
    file.write('Thrust (N) = '+str(thrust_full)+'\n')
    file.write('\n')
        
    # ------------------------------------------------------------------    
    #   Aerodynamics 2
    # ------------------------------------------------------------------
    fig = plt.figure("Aerodynamimmc Coefficients")
    clift_full=[]
    cdrag_full=[]
    
    for segment in results.segments.values():
        
        time   = segment.conditions.frames.inertial.time[:,0] / Units.min
        CLift  = segment.conditions.aerodynamics.lift_coefficient[:,0]
        clift_full.extend(CLift)
        CDrag  = segment.conditions.aerodynamics.drag_coefficient[:,0]
        cdrag_full.extend(CDrag)
        Drag   = -segment.conditions.frames.wind.drag_force_vector[:,0]
        Thrust = segment.conditions.frames.body.thrust_force_vector[:,0]

        axes = fig.add_subplot(3,1,1)
        axes.plot( time , CLift , 'bo-' )
        axes.set_xlabel('Time (min)')
        axes.set_ylabel('CL')
        axes.get_yaxis().get_major_formatter().set_scientific(False)
        axes.get_yaxis().get_major_formatter().set_useOffset(False)          
        axes.grid(True)
        
        axes = fig.add_subplot(3,1,2)
        axes.plot( time , CDrag , 'bo-' )
        axes.set_xlabel('Time (min)')
        axes.set_ylabel('CD')
        axes.get_yaxis().get_major_formatter().set_scientific(False)
        axes.get_yaxis().get_major_formatter().set_useOffset(False)          
        axes.grid(True)
        
        axes = fig.add_subplot(3,1,3)
        axes.plot( time , Drag   , 'bo-' )
        axes.plot( time , Thrust , 'ro-' )
        axes.set_xlabel('Time (min)')
        axes.set_ylabel('Drag and Thrust (N)')
        axes.get_yaxis().get_major_formatter().set_scientific(False)
        axes.get_yaxis().get_major_formatter().set_useOffset(False)  
        axes.grid(True)
        
    file.write('C_lift = '+str(clift_full)+'\n')
    file.write('C_drag = '+str(cdrag_full)+'\n')
    file.write('\n')
        
    
    # ------------------------------------------------------------------    
    #   Aerodynamics 2
    # ------------------------------------------------------------------
    fig = plt.figure("Drag Components")
    cdparasite_full=[]
    cdinduced_full=[]
    cdcompressible_full=[]
    cdmis_full=[]
    cdtotal_full=[]
    axes = plt.gca()    
    for i, segment in enumerate(results.segments.values()):
        
        time   = segment.conditions.frames.inertial.time[:,0] / Units.min
        drag_breakdown = segment.conditions.aerodynamics.drag_breakdown
        cdp = drag_breakdown.parasite.total[:,0]
        cdparasite_full.extend(cdp)
        cdi = drag_breakdown.induced.total[:,0]
        cdinduced_full.extend(cdi)
        cdc = drag_breakdown.compressible.total[:,0]
        cdcompressible_full.extend(cdc)
        cdm = drag_breakdown.miscellaneous.total[:,0]
        cdmis_full.extend(cdm)
        cd  = drag_breakdown.total[:,0]
        cdtotal_full.extend(cd)
        
        axes.plot( time , cdp , 'ko-', label='CD_parasite' )
        axes.plot( time , cdi , 'bo-', label='CD_induced' )
        axes.plot( time , cdc , 'go-', label='CD_compressible' )
        axes.plot( time , cdm , 'yo-', label='CD_miscellaneous' )
        axes.plot( time , cd  , 'ro-', label='CD_total'   )
        
        if i == 0:
            axes.legend(loc='upper center')
        
    axes.set_xlabel('Time (min)')
    axes.set_ylabel('CD')
    axes.grid(True)
    
    file.write('C_drag_parasite = '+str(cdparasite_full)+'\n')
    file.write('C_drag_induced = '+str(cdinduced_full)+'\n')
    file.write('C_drag_compressible = '+str(cdcompressible_full)+'\n')
    file.write('C_drag_miscellaneous = '+str(cdmis_full)+'\n')
    file.write('C_drag_total = '+str(cdtotal_full)+'\n')
    file.write('\n')
    
    # ------------------------------------------------------------------    
    #   Current Draw
    # ------------------------------------------------------------------
    plt.figure("Current Draw")
    curr_draw_HL_full=[]
    curr_draw_cruise_full=[]
    axes = plt.gca()
    i=0
    for segment in results.segments.values():     
        time     = segment.conditions.frames.inertial.time[:,0] / Units.min
        energy_lift = segment.conditions.propulsion.current_lift[:,0]
        curr_draw_HL_full.extend(energy_lift)
        energy_forward = segment.conditions.propulsion.current_forward[:,0]
        curr_draw_cruise_full.extend(energy_forward)
        axes.plot(time, energy_lift, 'ko-', label='HL Propellers' if i == 0 else "")
        axes.plot(time, energy_forward, 'bo-', label='Cruise Propellers' if i == 0 else "")
        i=i+1
        
    axes.set_xlabel('Time (mins)')
    axes.set_ylabel('Current Draw (Amps)')
    axes.get_yaxis().get_major_formatter().set_scientific(False)
    axes.get_yaxis().get_major_formatter().set_useOffset(False)   
    plt.ylim((0,500))
    plt.legend(loc='best')
    axes.grid(True)  
    file.write('CurrentDraw Cruise (Amps) = '+str(curr_draw_cruise_full)+'\n')
    file.write('CurrentDraw Lift (Amps) = '+str(curr_draw_HL_full)+'\n')
    file.write('\n')
    
    # ------------------------------------------------------------------    
    #   Motor RPM
    # ------------------------------------------------------------------
    plt.figure("Motor RPM")
    axes = plt.gca()
    i=0
    rpm_HL_full=[]
    rpm_cruise_full=[]
    for segment in results.segments.values():   
        time     = segment.conditions.frames.inertial.time[:,0] / Units.min
        energy_lift = segment.conditions.propulsion.rpm_lift[:,0]
        rpm_HL_full.extend(energy_lift)
        energy_forward = segment.conditions.propulsion.rpm_forward[:,0]
        rpm_cruise_full.extend(energy_forward)
        axes.plot(time, energy_lift, 'ko-', label='HL Propellers' if i == 0 else "")
        axes.plot(time, energy_forward, 'bo-', label='Cruise Propellers' if i == 0 else "")
        i=i+1
    
    axes.set_xlabel('Time (mins)')
    axes.set_ylabel('Motor RPM')
    axes.get_yaxis().get_major_formatter().set_scientific(False)
    axes.get_yaxis().get_major_formatter().set_useOffset(False) 
    plt.legend(loc='best')
    axes.grid(True)
    file.write('RPM Cruise = '+str(rpm_cruise_full)+'\n')
    file.write('RPM Lift = '+str(rpm_HL_full)+'\n')
    file.write('\n')

    # ------------------------------------------------------------------
    #   Flight Conditions
    # ------------------------------------------------------------------
    fig = plt.figure("Flight Conditions")
    altitude2_full=[]
    mach_full=[]
    aoa_full=[]
    for segment in results.segments.values():

        time     = segment.conditions.frames.inertial.time[:,0] / Units.min
        altitude = segment.conditions.freestream.altitude[:,0] / Units.km
        altitude2_full.extend(altitude)
        mach     = segment.conditions.freestream.mach_number[:,0]
        mach_full.extend(mach)
        aoa      = segment.conditions.aerodynamics.angle_of_attack[:,0] / Units.deg
        aoa_full.extend(aoa)

        axes = fig.add_subplot(3,1,1)
        axes.plot( time , aoa , 'bo-' )
        axes.set_ylabel('Angle of Attack (deg)')
        axes.get_yaxis().get_major_formatter().set_scientific(False)
        axes.get_yaxis().get_major_formatter().set_useOffset(False)          
        axes.grid(True)

        axes = fig.add_subplot(3,1,2)
        axes.plot( time , altitude , 'bo-' )
        axes.set_ylabel('Altitude (km)')
        axes.get_yaxis().get_major_formatter().set_scientific(False)
        axes.get_yaxis().get_major_formatter().set_useOffset(False)          
        axes.grid(True)

        axes = fig.add_subplot(3,1,3)
        axes.plot( time , mach, 'bo-' )
        axes.set_xlabel('Time (min)')
        axes.set_ylabel('Mach Number (-)')
        axes.get_yaxis().get_major_formatter().set_scientific(False)
        axes.get_yaxis().get_major_formatter().set_useOffset(False)          
        axes.grid(True)   
        
    file.write('Altitude (km) = '+str(altitude2_full)+'\n')
    file.write('Mach = '+str(mach_full)+'\n')
    file.write('AoA (deg) = '+str(aoa_full)+'\n')
    file.write('\n')
        
    
    # ------------------------------------------------------------------    
    #  Charging Power and Battery Energy
    # ------------------------------------------------------------------
    
    fig = plt.figure("Electric Outputs")
    bat_draw_full=[]
    bat_energy_full=[]
    for segment in results.segments.values():
        
        time   = segment.conditions.frames.inertial.time[:,0] / Units.min 
        charge = segment.conditions.propulsion.battery_draw[:,0]
        bat_draw_full.extend(charge)
        energy = segment.conditions.propulsion.battery_energy[:,0] / Units.MJ
        bat_energy_full.extend(energy)
        
        axes = fig.add_subplot(3,1,2)
        axes.plot( time , charge , 'bo-' )
        axes.set_ylabel('Charging Power (W)')
        axes.grid(True)
        
        axes = fig.add_subplot(3,1,3)
        axes.plot( time , energy , 'bo-' )
        axes.set_xlabel('Time (min)')
        axes.set_ylabel('Battery Energy (MJ)')
        axes.grid(True)
        
    file.write('Battery Draw/Charging Power (W) = '+str(bat_draw_full)+'\n')
    file.write('Battery Energy (MJ) = '+str(bat_energy_full)+'\n')
    file.write('\n') 
    
    
    cp_lift=[]
    cp_forward=[]
    ct_lift=[]
    ct_forward=[]
    vel_sound_full=[]
    vel_sound_full_ms=[]
    rho_full=[] 
    
    for segment in results.segments.values():
        
        cp_lift_aux   = segment.conditions.propulsion.propeller_power_coefficient_lift[:,0]
        cp_lift.extend(cp_lift_aux)
        cp_forward_aux = segment.conditions.propulsion.propeller_power_coefficient[:,0]
        cp_forward.extend(cp_forward_aux)
        ct_lift_aux    = segment.conditions.propulsion.propeller_thrust_coefficient_lift[:,0]
        ct_lift.extend(ct_lift_aux)
        ct_forward_aux = segment.conditions.propulsion.propeller_thrust_coefficient_forward[:,0]
        ct_forward.extend(ct_forward_aux)
        vel_sound_aux = segment.conditions.freestream.speed_of_sound[:,0]
        vel_sound_full.extend(vel_sound_aux)
        vel_sound_aux_ms = segment.conditions.freestream.speed_of_sound[:,0] / Units['m/s']
        vel_sound_full_ms.extend(vel_sound_aux_ms) 
        rho_aux = segment.conditions.freestream.density[:,0]
        rho_full.extend(rho_aux)
        
        
    file.write('Cp_lift = '+str(cp_lift)+'\n')
    file.write('Cp_forward = '+str(cp_forward)+'\n')
    file.write('Ct_lift = '+str(ct_lift)+'\n')
    file.write('Ct_forward = '+str(ct_forward)+'\n')
    file.write('Vel Sound (default) = '+str(vel_sound_full)+'\n')
    file.write('Vel Sound (m/s) = '+str(vel_sound_full_ms)+'\n')
    file.write('Rho (default) = '+str(rho_full)+'\n')
    file.write('\n')
    
    
    file.close()
    
        
    return
if __name__ == '__main__': 
    main()    
    plt.show()