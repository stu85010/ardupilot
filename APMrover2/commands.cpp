#include "Rover.h"
/*
 *  set_auto_WP - sets the target location the vehicle should drive to in Auto mode
 */
void Rover::set_auto_WP(const struct Location& loc)
{
    // copy the current WP into the OldWP slot
    // ---------------------------------------
    prev_WP = next_WP;

    // Load the next_WP slot
    // ---------------------
    next_WP = loc;

    // are we already past the waypoint? This happens when we jump
    // waypoints, and it can cause us to skip a waypoint. If we are
    // past the waypoint when we start on a leg, then use the current
    // location as the previous waypoint, to prevent immediately
    // considering the waypoint complete
    if (location_passed_point(current_loc, prev_WP, next_WP)) {
        gcs_send_text(MAV_SEVERITY_NOTICE, "Resetting previous WP");
        prev_WP = current_loc;
    }

    // this is handy for the groundstation
    wp_totalDistance = get_distance(current_loc, next_WP);
    wp_distance      = wp_totalDistance;
}

void Rover::set_guided_WP(const struct Location& loc)
{
    guided_mode = Guided_WP;
    // copy the current location into the OldWP slot
    // ---------------------------------------
    prev_WP = current_loc;

    // Load the next_WP slot
    // ---------------------
    next_WP = loc;
    rover.guided_control.target_speed = g.speed_cruise;
    // this is handy for the groundstation
    wp_totalDistance = get_distance(current_loc, next_WP);
    wp_distance      = wp_totalDistance;

    rover.rtl_complete = false;
}

void Rover::set_guided_velocity(float target_steer_speed, float target_speed)
{
    guided_mode = Guided_Velocity;
    rover.guided_control.target_steer_speed = target_steer_speed;
    rover.guided_control.target_speed = target_speed;

    next_WP = current_loc;
    lateral_acceleration = 0.0f;
    // this is handy for the groundstation
    wp_totalDistance = 0;
    wp_distance      = 0.0f;

    rover.rtl_complete = false;
}

// run this at setup on the ground
// -------------------------------
void Rover::init_home()
{
    if (!have_position) {
        // we need position information
        return;
    }

    gcs_send_text(MAV_SEVERITY_INFO, "Init HOME");

    ahrs.set_home(gps.location());
    home_is_set = HOME_SET_NOT_LOCKED;
    Log_Write_Home_And_Origin();
    GCS_MAVLINK::send_home_all(gps.location());

    // Save Home to EEPROM
    mission.write_home_to_storage();

    // Save prev loc
    // -------------
    next_WP = prev_WP = home;

    // Load home for a default guided_WP
    // -------------
    set_guided_WP(home);
}

void Rover::restart_nav()
{
    g.pidSpeedThrottle.reset_I();
    prev_WP = current_loc;
    mission.start_or_resume();
}

/*
  update home location from GPS
  this is called as long as we have 3D lock and the arming switch is
  not pushed
*/
void Rover::update_home()
{
    if (home_is_set == HOME_SET_NOT_LOCKED) {
        Location loc;
        if (ahrs.get_position(loc)) {
            ahrs.set_home(loc);
            Log_Write_Home_And_Origin();
            GCS_MAVLINK::send_home_all(gps.location());
        }
    }
    barometer.update_calibration();
}
