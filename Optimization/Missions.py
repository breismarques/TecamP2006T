# Missions.py

# Python Imports
import numpy as np
import pylab as plt
from subprocess import call

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
from SUAVE.Input_Output.OpenVSP import vsp_write
#from SUAVE.Input_Output.OpenVSP import vspaero

# ----------------------------------------------------------------------
#   Define the Mission
# ----------------------------------------------------------------------

def setup(analyses,vehicle):
    
    # the mission container
    missions = SUAVE.Analyses.Mission.Mission.Container()   
    missions.mission = mission_setup(analyses,vehicle)

    return missions  

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

    
    segment.state.numerics.number_control_points = 4
    segment.altitude_start = 0.0   * Units.km
    segment.altitude_end   = 3.0   * Units.km
    segment.air_speed      = 125.0 * Units['m/s']
    segment.climb_rate     = 6.0   * Units['m/s']
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
    segment.altitude_end   = 8.0   * Units.km
    segment.air_speed      = 190.0 * Units['m/s']
    segment.climb_rate     = 3.0   * Units['m/s']

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
    segment.altitude       = 8. * Units.km
    segment.air_speed  = 200.0 * Units['m/s']
    segment.distance       = 150. * Units.nautical_miles
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
    segment.altitude_end = 3.0   * Units.km
    segment.air_speed    = 180.0 * Units['m/s']
    segment.descent_rate = 4.5   * Units['m/s']
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
    segment.air_speed      = 145.0 * Units['m/s']
    segment.climb_rate     = 3.0   * Units['m/s']

    # add to misison
    mission.append_segment(segment)
    

    # ------------------------------------------------------------------    
    #   Mission definition complete    
    # ------------------------------------------------------------------
    
    return mission

#def missions_setup(base_mission):

    # the mission container
#    missions = SUAVE.Analyses.Mission.Mission.Container()

    # ------------------------------------------------------------------
    #   Base Mission
    # ------------------------------------------------------------------

#    missions.base = base_mission

#    return missions  