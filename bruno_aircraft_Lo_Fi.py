# Vehicles.py

# Python Imports
import numpy as np
import pylab as plt
from subprocess import call
import time
import timeit
import os
import re

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
    mission = analyses.missions.base
    results = mission.evaluate()
    
    # plot results    
    plot_mission(results)
    
    initial_time=configs.base.fuselages.fuselage.time
    
    time1 = time.time()
            
    print 'The total time to run the SIMULATION: '+ str(time1-initial_time) + '  Seconds'
    
    #call(["/Users/Bruno/OpenVSP/build/_CPack_Packages/MacOS/ZIP/OpenVSP-3.13.3-MacOS/vsp","open","X57Opt.vsp3"])
    

    return

# ----------------------------------------------------------------------
#   Analysis Setup
# ----------------------------------------------------------------------

def full_setup():

    # vehicle data
    vehicle  = vehicle_setup()
    configs  = configs_setup(vehicle)

    # vehicle analyses
    configs_analyses = analyses_setup(configs)
    

    # mission analyses
    mission  = mission_setup(configs_analyses,configs)
    missions_analyses = missions_setup(mission)

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

# ----------------------------------------------------------------------
#   Define the Vehicle
# ----------------------------------------------------------------------

def vehicle_setup():
    
    # Take values from optimization
    
    cwd = os.getcwd()
    
    with open(cwd+'/Optimization_Lo_Fid/results.txt', 'r') as f:
        lines = f.read().splitlines()
        last_line = lines[-1]
        
    inputs = last_line.split(',')
    
    inputs_opt=np.zeros(24)
    inputs_opt[0]=float(inputs[2][11:])
    inputs_opt[1]=float(inputs[3])
    inputs_opt[2]=float(inputs[4])
    inputs_opt[3]=float(inputs[5])
    inputs_opt[4]=float(inputs[6])
    inputs_opt[5]=float(inputs[7])
    inputs_opt[6]=float(inputs[8])
    inputs_opt[7]=float(inputs[9])
    inputs_opt[8]=float(inputs[10])
    inputs_opt[9]=float(inputs[11])
    inputs_opt[10]=float(inputs[12])
    inputs_opt[11]=float(inputs[13])
    inputs_opt[12]=float(inputs[14])
    inputs_opt[13]=float(inputs[15])
    inputs_opt[14]=float(inputs[16])
    inputs_opt[15]=float(inputs[17])
    inputs_opt[16]=float(inputs[18])
    inputs_opt[17]=float(inputs[19])
    inputs_opt[18]=float(inputs[20])
    inputs_opt[19]=float(inputs[21])
    inputs_opt[20]=float(inputs[22])
    inputs_opt[21]=float(inputs[23])
    inputs_opt[22]=float(inputs[24])
    inputs_opt[23]=float(inputs[25][:-1])
    
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
    landing_gear = SUAVE.Components.Landing_Gear.Landing_Gear()
    landing_gear.tag = "main_landing_gear"
    
    landing_gear.main_tire_diameter = 0.423 * Units.m
    landing_gear.nose_tire_diameter = 0.3625 * Units.m
    landing_gear.main_strut_length  = 0.4833 * Units.m
    landing_gear.nose_strut_length  = 0.3625 * Units.m
    landing_gear.main_units  = 2    #number of main landing gear units
    landing_gear.nose_units  = 1    #number of nose landing gear
    landing_gear.main_wheels = 1    #number of wheels on the main landing gear
    landing_gear.nose_wheels = 1    #number of wheels on the nose landing gear      
    vehicle.landing_gear = landing_gear
    
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
    fuselage.differential_pressure = 0.0 * Units.pascal # Maximum differential pressure
    
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
    
    wing.thickness_to_chord      = 0.15
    wing.taper                   = 0.9
    wing.spans.projected         = 9.4*inputs_opt[0] * Units.meter
    wing.chords.root             = 2.0*inputs_opt[1] * Units.meter
    wing.chords.tip              = wing.chords.root*wing.taper
    wing.chords.mean_aerodynamic = 0.6 * Units.meter
    wing.areas.reference         = (wing.chords.root+wing.chords.tip)*wing.spans.projected/2  
    wing.twists.root             = 0. * Units.degrees
    wing.twists.tip              = 0. * Units.degrees
    wing.dihedral= 1. * Units.degrees
    wing.origin                  = [2.986,0,1.077] # meters
    wing.sweeps.leading_edge     = 1.9 * Units.deg
    wing.aspect_ratio            = (wing.spans.projected*wing.spans.projected)/wing.areas.reference
    wing.span_efficiency         = 0.99*(1-0.0407*(fuselage.width/wing.spans.projected)-1.792*((fuselage.width/wing.spans.projected)**2))
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
    
    wing.flaps.chord      =  0.20   
    wing.flaps.span_start =  0.1053
    wing.flaps.span_end   =  0.6842
    wing.flaps.type       = 'single_slotted'

    # add to vehicle
    vehicle.append_component(wing)

    # ------------------------------------------------------------------        
    #  Horizontal Stabilizer
    # ------------------------------------------------------------------        
    
    wing = SUAVE.Components.Wings.Wing()
    wing.tag = 'horizontal_stabilizer'
    
    wing.aspect_ratio            = 4.193     
    wing.sweeps.quarter_chord    = 0.0 * Units.deg
    wing.thickness_to_chord      = 0.12
    wing.taper                   = 1.0
    wing.span_efficiency         = 0.733
    wing.spans.projected         = 3.3 * Units.meter
    wing.chords.root             = 0.787 * Units.meter
    wing.chords.tip              = 0.787 * Units.meter
    wing.chords.mean_aerodynamic = (wing.chords.root*(2.0/3.0)*((1.0+wing.taper+wing.taper**2.0)/(1.0+wing.taper))) * Units.meter
    wing.areas.reference         = 2.5971 * Units['meters**2']  
    wing.areas.exposed           = 4.0 * Units['meters**2']  
    wing.areas.wetted            = 4.0 * Units['meters**2']  
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

    wing.aspect_ratio            = 1.407
    wing.sweeps.quarter_chord    = 38.75 * Units.deg
    wing.thickness_to_chord      = 0.12
    wing.taper                   = 1.0
    wing.span_efficiency         = -0.107
    wing.spans.projected         = 1.574 * Units.meter
    wing.chords.root             = 1.2 * Units.meter
    wing.chords.tip              = 0.497 * Units.meter
    wing.chords.mean_aerodynamic = (wing.chords.root*(2.0/3.0)*((1.0+wing.taper+wing.taper**2.0)/(1.0+wing.taper))) * Units.meter
    wing.areas.reference         = 1.761 * Units['meters**2']  
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
    net.number_of_engines_lift    = 14
    net.number_of_engines_forward = 2
    net.thrust_angle_lift         = 0.0 * Units.degrees
    net.thrust_angle_forward      = 0.0 * Units.degrees
    net.voltage           = 461.0*inputs_opt[15]  #461.
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
    
    prop_forward = SUAVE.Components.Energy.Converters.Propeller_Lo_Fid()
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
    
    prop_lift = SUAVE.Components.Energy.Converters.Propeller_Lo_Fid()
    prop_lift.number_blades       = 5.0
    prop_lift.propulsive_efficiency = 0.85
    prop_lift.freestream_velocity = 1. * Units['m/s'] # freestream m/s
    prop_lift.angular_velocity    = 2750 * Units['rpm'] # For 20x10 prop
    prop_lift.tip_radius          = 0.26 * Units.meter  #0.2880360
    prop_lift.hub_radius          = 0.07772400 * Units.meter
    prop_lift.design_Cl           = 0.8
    prop_lift.design_altitude     = 0.0 * Units.meter
    prop_lift.design_thrust       = 0.0 * Units.newton
    prop_lift.design_power        = 10000. * Units.watts 
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
    
    kv                         = 180.0*inputs_opt[16] * Units['rpm/volt'] # RPM/volt is standard
    motor                      = size_from_kv(motor, kv)    
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
    
    kv                         = 90.0*inputs_opt[17] * Units['rpm/volt'] # RPM/volt is standard
    motor                      = size_from_kv(motor, kv)    
    motor.gear_ratio           = 1. # Gear ratio, no gearbox
    motor.gearbox_efficiency   = .98 # Gear box efficiency, no gearbox
    motor.motor_efficiency     = 0.825;
    net.motor_lift                  = motor
    
    # Component 4 - the Payload
    payload = SUAVE.Components.Energy.Peripherals.Payload()
    payload.power_draw           = 50.0*inputs_opt[22] * Units.watts 
    payload.mass_properties.mass = 5.0 * Units.kg
    net.payload                  = payload
    
    # Component 5 - the Avionics
    avionics = SUAVE.Components.Energy.Peripherals.Avionics()
    avionics.power_draw = 50.0*inputs_opt[23] * Units.watts
    net.avionics        = avionics      

    # Component 6 - the Battery
    bat = SUAVE.Components.Energy.Storages.Batteries.Constant_Mass.Lithium_Ion()
    bat.mass_properties.mass = 358.33 * Units.kg
    bat.specific_energy      = 4500.0*inputs_opt[18] * Units.Wh/Units.kg  #192.84
    bat.specific_power       = 0.837*inputs_opt[19] * Units.kW/Units.kg  #0.837
    bat.resistance           = 0.0153*inputs_opt[21]
    bat.max_voltage          = 60000.0*inputs_opt[20]   #10000.
    initialize_from_mass(bat,bat.mass_properties.mass)
    net.battery              = bat
   
    
    # ------------------------------------------------------------------
    #   Vehicle Definition Complete
    # ------------------------------------------------------------------
    
    # add the energy network to the vehicle
    vehicle.append_component(net)
    
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
    #   Cruise Configuration
    # ------------------------------------------------------------------
    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'cruise'
    config.propulsors.propulsor.number_of_engines_lift = 0
    configs.append(config)

    # ------------------------------------------------------------------
    #   Takeoff Configuration
    # ------------------------------------------------------------------
    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'takeoff'
    #config.wings['main_wing'].flaps.angle = 30. * Units.deg
    config.max_lift_coefficient_factor    = 1.

    configs.append(config)
    
    # ------------------------------------------------------------------
    #   Climb Configuration
    # ------------------------------------------------------------------
    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'climb'
    #config.wings['main_wing'].flaps.angle = 30. * Units.deg

    configs.append(config)
    
    # ------------------------------------------------------------------
    #   Descent Configuration
    # ------------------------------------------------------------------
    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'descent'
    #config.wings['main_wing'].flaps.angle = 30. * Units.deg

    configs.append(config)
    
    # ------------------------------------------------------------------
    #   Cutback Configuration
    # ------------------------------------------------------------------
    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'cutback'
    #config.wings['main_wing'].flaps.angle = 30. * Units.deg
    #config.max_lift_coefficient_factor    = 1. #0.95

    configs.append(config)    

    # ------------------------------------------------------------------
    #   Landing Configuration
    # ------------------------------------------------------------------

    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'landing'

    #config.wings['main_wing'].flaps.angle = 40. * Units.deg  
    #config.max_lift_coefficient_factor    = 1. #0.95

    configs.append(config)

    # ------------------------------------------------------------------
    #   Short Field Takeoff Configuration
    # ------------------------------------------------------------------ 

    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'short_field_takeoff'
    
    #config.wings['main_wing'].flaps.angle = 30. * Units.deg
    #config.max_lift_coefficient_factor    = 1. #0.95
  
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

# ----------------------------------------------------------------------
#   Define the Mission
# ----------------------------------------------------------------------

def mission_setup(analyses,vehicle):
    
    # Take values from optimization
    
    cwd = os.getcwd()
    
    with open(cwd+'/Optimization_Lo_Fid/results.txt', 'r') as f:
        lines = f.read().splitlines()
        last_line = lines[-1]
        
    inputs = last_line.split(',')
    
    inputs_opt=np.zeros(24)
    inputs_opt[0]=float(inputs[2][11:])
    inputs_opt[1]=float(inputs[3])
    inputs_opt[2]=float(inputs[4])
    inputs_opt[3]=float(inputs[5])
    inputs_opt[4]=float(inputs[6])
    inputs_opt[5]=float(inputs[7])
    inputs_opt[6]=float(inputs[8])
    inputs_opt[7]=float(inputs[9])
    inputs_opt[8]=float(inputs[10])
    inputs_opt[9]=float(inputs[11])
    inputs_opt[10]=float(inputs[12])
    inputs_opt[11]=float(inputs[13])
    inputs_opt[12]=float(inputs[14])
    inputs_opt[13]=float(inputs[15])
    inputs_opt[14]=float(inputs[16])
    inputs_opt[15]=float(inputs[17])
    inputs_opt[16]=float(inputs[18])
    inputs_opt[17]=float(inputs[19])
    inputs_opt[18]=float(inputs[20])
    inputs_opt[19]=float(inputs[21])
    inputs_opt[20]=float(inputs[22])
    inputs_opt[21]=float(inputs[23])
    inputs_opt[22]=float(inputs[24])
    inputs_opt[23]=float(inputs[25][:-1])

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

    
    segment.state.numerics.number_control_points = 4
    segment.altitude_start = 0.0   * Units.km
    segment.altitude_end   = 3.0*inputs_opt[8]   * Units.km
    segment.air_speed      = 125.0*inputs_opt[4] * Units['m/s']
    segment.climb_rate     = 6.0*inputs_opt[11]   * Units['m/s']
    segment.battery_energy = vehicle.base.propulsors.propulsor.battery.max_energy

    # add to misison
    mission.append_segment(segment)
    
    # ------------------------------------------------------------------
    #   Second Climb Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------
    
    # unpack Segments module
    Segments = SUAVE.Analyses.Mission.Segments

    segment = Segments.Climb.Constant_Speed_Constant_Rate_VSP(no_lift_segment)
    segment.tag = "climb_2"

    segment.analyses.extend(analyses.climb)

    
    segment.state.numerics.number_control_points = 4
    segment.altitude_end   = 8.0*inputs_opt[9]  * Units.km
    segment.air_speed      = 190.0*inputs_opt[5] * Units['m/s']
    segment.climb_rate     = 3.0*inputs_opt[12]   * Units['m/s']

    # add to misison
    mission.append_segment(segment)
    
    
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
    segment.state.numerics.number_control_points = 4
    segment.altitude       = 8.0*inputs_opt[9] * Units.km
    segment.air_speed  = 200.0*inputs_opt[3] * Units['m/s']
    segment.distance       = 150.0*inputs_opt[2] * Units.nautical_miles
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

    segment.analyses.extend(analyses.descent)

    
    segment.state.numerics.number_control_points = 4
    segment.altitude_end = 3.0*inputs_opt[10]   * Units.km
    segment.air_speed    = 180.0*inputs_opt[6] * Units['m/s']
    segment.descent_rate = 4.5*inputs_opt[13]   * Units['m/s']
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

    
    segment.state.numerics.number_control_points = 4
    segment.altitude_end   = 0.0   * Units.km
    segment.air_speed      = 145.0*inputs_opt[7] * Units['m/s']
    segment.descent_rate     = 3.0*inputs_opt[14]   * Units['m/s']

    # add to misison
    mission.append_segment(segment)
    

    # ------------------------------------------------------------------    
    #   Mission definition complete    
    # ------------------------------------------------------------------
    
    return mission

def missions_setup(base_mission):

    # the mission container
    missions = SUAVE.Analyses.Mission.Mission.Container()

    # ------------------------------------------------------------------
    #   Base Mission
    # ------------------------------------------------------------------

    missions.base = base_mission

    return missions  

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
    for segment in results.segments.values():
        time = segment.conditions.frames.inertial.time[:,0] / Units.min
        eta  = segment.conditions.propulsion.throttle[:,0]
        eta_lift = segment.conditions.propulsion.lift_throttle[:,0]
        
        axes.plot(time, eta, 'ko-', label='Cruise Throttle' if i == 0 else "")
        axes.plot(time, eta_lift, 'bo-', label='HL Throttle' if i == 0 else "")
        i=i+1
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
    for segment in results.segments.values():     
        time     = segment.conditions.frames.inertial.time[:,0] / Units.min
        altitude = segment.conditions.freestream.altitude[:,0] / Units.km
        axes.plot(time, altitude, 'bo-')
    axes.set_xlabel('Time (mins)')
    axes.set_ylabel('Altitude (km)')
    axes.grid(True)    
    
    # ------------------------------------------------------------------    
    #   Aerodynamics
    # ------------------------------------------------------------------
    fig = plt.figure("Aerodynamic Forces")
    for segment in results.segments.values():
        
        time   = segment.conditions.frames.inertial.time[:,0] / Units.min
        Lift   = -segment.conditions.frames.wind.lift_force_vector[:,2]
        Drag   = -segment.conditions.frames.wind.drag_force_vector[:,0]
        Thrust = segment.conditions.frames.body.thrust_force_vector[:,0]

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
        
    # ------------------------------------------------------------------    
    #   Aerodynamics 2
    # ------------------------------------------------------------------
    fig = plt.figure("Aerodynamic Coefficients")
    for segment in results.segments.values():
        
        time   = segment.conditions.frames.inertial.time[:,0] / Units.min
        CLift  = segment.conditions.aerodynamics.lift_coefficient[:,0]
        CDrag  = segment.conditions.aerodynamics.drag_coefficient[:,0]
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
        
    
    # ------------------------------------------------------------------    
    #   Aerodynamics 2
    # ------------------------------------------------------------------
    fig = plt.figure("Drag Components")
    axes = plt.gca()    
    for i, segment in enumerate(results.segments.values()):
        
        time   = segment.conditions.frames.inertial.time[:,0] / Units.min
        drag_breakdown = segment.conditions.aerodynamics.drag_breakdown
        cdp = drag_breakdown.parasite.total[:,0]
        cdi = drag_breakdown.induced.total[:,0]
        cdc = drag_breakdown.compressible.total[:,0]
        cdm = drag_breakdown.miscellaneous.total[:,0]
        cd  = drag_breakdown.total[:,0]
        
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
    
    # ------------------------------------------------------------------    
    #   Current Draw
    # ------------------------------------------------------------------
    plt.figure("Current Draw")
    axes = plt.gca()
    i=0
    for segment in results.segments.values():     
        time     = segment.conditions.frames.inertial.time[:,0] / Units.min
        energy_lift = segment.conditions.propulsion.current_lift[:,0]
        energy_forward = segment.conditions.propulsion.current_forward[:,0]
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
    
    # ------------------------------------------------------------------    
    #   Motor RPM
    # ------------------------------------------------------------------
    plt.figure("Motor RPM")
    axes = plt.gca()
    i=0
    for segment in results.segments.values():   
        time     = segment.conditions.frames.inertial.time[:,0] / Units.min
        energy_lift = segment.conditions.propulsion.rpm_lift[:,0]
        energy_forward = segment.conditions.propulsion.rpm_forward[:,0]
        axes.plot(time, energy_lift, 'ko-', label='HL Propellers' if i == 0 else "")
        axes.plot(time, energy_forward, 'bo-', label='Cruise Propellers' if i == 0 else "")
        i=i+1
    
    axes.set_xlabel('Time (mins)')
    axes.set_ylabel('Motor RPM')
    axes.get_yaxis().get_major_formatter().set_scientific(False)
    axes.get_yaxis().get_major_formatter().set_useOffset(False) 
    plt.legend(loc='best')
    axes.grid(True)

    # ------------------------------------------------------------------
    #   Flight Conditions
    # ------------------------------------------------------------------
    fig = plt.figure("Flight Conditions")
    for segment in results.segments.values():

        time     = segment.conditions.frames.inertial.time[:,0] / Units.min
        altitude = segment.conditions.freestream.altitude[:,0] / Units.km
        mach     = segment.conditions.freestream.mach_number[:,0]
        aoa      = segment.conditions.aerodynamics.angle_of_attack[:,0] / Units.deg

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
        
    
    # ------------------------------------------------------------------    
    #  Charging Power and Battery Energy
    # ------------------------------------------------------------------
    
    fig = plt.figure("Electric Outputs")
    for segment in results.segments.values():
        
        time   = segment.conditions.frames.inertial.time[:,0] / Units.min 
        charge = segment.conditions.propulsion.battery_draw[:,0]
        energy = segment.conditions.propulsion.battery_energy[:,0] / Units.MJ
        
        axes = fig.add_subplot(3,1,2)
        axes.plot( time , charge , 'bo-' )
        axes.set_ylabel('Charging Power (W)')
        axes.grid(True)
        
        axes = fig.add_subplot(3,1,3)
        axes.plot( time , energy , 'bo-' )
        axes.set_xlabel('Time (min)')
        axes.set_ylabel('Battery Energy (MJ)')
        axes.grid(True)  
    
    return     
if __name__ == '__main__': 
    main()    
    plt.show()