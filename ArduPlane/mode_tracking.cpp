#include "mode.h"
#include "Plane.h"

/*
  TRACKING flight mode
  ====================
  Controls roll and pitch from external tracking errors received via
  the custom TRACKING MAVLink message (ID 229).

  errorx/errory arrive normalised in [-1, 1] and are converted to
  radians by GCS_MAVLink_Plane before being passed here.

  Roll control  (PID on errorx → nav_roll_cd):
    Tunable via parameters TRK_ROLL_P / _I / _D / _IMAX
    Default: P=200 cd/deg, I=10, D=5, imax=3000 cd

  Pitch control (PID on errory → nav_pitch_cd):
    Tunable via parameters TRK_PTCH_P / _I / _D / _IMAX
    Default: P=100 cd/deg, I=500, D=0, imax=3000 cd

  Throttle: constant TRIM_THROTTLE percent.
  Timeout:  if no message within TRACKING_TIMEOUT_MS, both PIDs are
            reset and roll/pitch commands are zeroed (hold level until
            signal returns).
*/



// ── _enter ────────────────────────────────────────────────────────────────────
bool ModeTracking::_enter()
{
    _errorx_rad     = 0.0f;
    _errory_rad     = 0.0f;
    _last_msg_ms    = 0;
    _prev_update_ms = AP_HAL::millis();
    _was_timed_out  = false;
    plane.g2.tracking_roll_pid.reset_I();
    plane.g2.tracking_roll_pid.reset_filter();
    plane.g2.tracking_pitch_pid.reset_I();
    plane.g2.tracking_pitch_pid.reset_filter();

    gcs().send_text(MAV_SEVERITY_INFO, "Tracking: active");
    return true;
}


// ── _exit ─────────────────────────────────────────────────────────────────────
void ModeTracking::_exit()
{
    plane.g2.tracking_roll_pid.reset_I();
    plane.g2.tracking_roll_pid.reset_filter();
    plane.g2.tracking_pitch_pid.reset_I();
    plane.g2.tracking_pitch_pid.reset_filter();
    gcs().send_text(MAV_SEVERITY_INFO, "Tracking: exit");
}


// ── handle_tracking_error ─────────────────────────────────────────────────────
// Called by GCS_MAVLink_Plane::handle_tracking_message() with values already
// converted from normalised [-1,1] to radians (× TRACKING_MAX_DELTA_RAD).
void ModeTracking::handle_tracking_error(float errorx_rad, float errory_rad)
{
    _errorx_rad  = errorx_rad;
    _errory_rad  = errory_rad;
    _last_msg_ms = AP_HAL::millis();
}


// ── update ────────────────────────────────────────────────────────────────────
void ModeTracking::update()
{
    const uint32_t now_ms = AP_HAL::millis();
    const float    dt_s   = constrain_float((now_ms - _prev_update_ms) * 1e-3f,
                                            0.001f, 0.5f);
    _prev_update_ms = now_ms;

    const uint32_t timeout_ms = (uint32_t)plane.g2.tracking_timeout_ms.get();
    const bool timed_out = (_last_msg_ms == 0) ||
                           (now_ms - _last_msg_ms > timeout_ms);

    bool ex_in_deadband = true;   // default: treat as centred when timed out

    if (timed_out) {
        // No recent tracking signal: freeze nav_roll_cd / nav_pitch_cd at
        // their last commanded values so the aircraft holds its last attitude.
        if (!_was_timed_out) {
            // First cycle of timeout: flush integrators AND derivative filters
            // to prevent wind-up and stale derivative carry-over.
            plane.g2.tracking_roll_pid.reset_I();
            plane.g2.tracking_roll_pid.reset_filter();
            plane.g2.tracking_pitch_pid.reset_I();
            plane.g2.tracking_pitch_pid.reset_filter();
        }
        _was_timed_out = true;
    } else {
        _was_timed_out = false;
        const float deadband_rad = plane.g2.tracking_deadband_deg.get() * (M_PI / 180.0f);

        // ── Roll PID ─────────────────────────────────────────────────────────
        // errorx > 0 → target is to the right → roll right (positive bank).
        // Reset I when inside deadband to prevent integrator wind-up.
        const float ex = fabsf(_errorx_rad) > deadband_rad ? _errorx_rad : 0.0f;
        ex_in_deadband = (ex == 0.0f);
        if (ex == 0.0f) {
            plane.g2.tracking_roll_pid.reset_I();
            // In deadband: use gyro roll rate to actively damp any rolling
            // motion and drive toward wings-level.  Each deg/s of roll rate
            // commands an opposing 50 cdeg (0.5 deg) of bank target so the
            // attitude controller fights the rotation rather than coasting.
            const float roll_rate_dps = degrees(ahrs.get_gyro().x);
            const float damp_cd       = -(roll_rate_dps * 50.0f);
            plane.nav_roll_cd = constrain_int32((int32_t)damp_cd,
                                                -plane.roll_limit_cd,
                                                 plane.roll_limit_cd);
        } else {
            const float roll_cd = plane.g2.tracking_roll_pid.update_all(degrees(ex), 0.0f, dt_s);
            plane.nav_roll_cd   = constrain_int32((int32_t)roll_cd,
                                                  -plane.roll_limit_cd,
                                                   plane.roll_limit_cd);
        }

        // ── Pitch PID ────────────────────────────────────────────────────────
        // errory > 0 → target is above → pitch up (positive setpoint).
        // TRK_PITCH_OFFSET adds a constant bias (converted to rad) so the
        // aircraft can be trimmed toward the target without retuning the PID.
        // Reset I when inside deadband to prevent integrator wind-up.
        const float pitch_offset_rad = plane.g2.tracking_pitch_offset.get() * (M_PI / 180.0f);
        const float ey_raw = fabsf(_errory_rad) > deadband_rad ? _errory_rad : 0.0f;
        const float ey     = ey_raw - pitch_offset_rad;
        if (ey_raw == 0.0f) {
            plane.g2.tracking_pitch_pid.reset_I();
        }
        const float pitch_cd = plane.g2.tracking_pitch_pid.update_all(degrees(ey), 0.0f, dt_s);
        plane.nav_pitch_cd   = constrain_int32((int32_t)pitch_cd,
                                               (int32_t)(plane.pitch_limit_min * 100),
                                               plane.aparm.pitch_limit_max.get() * 100);
    }

    plane.update_load_factor();

    // ── Throttle ─────────────────────────────────────────────────────────────
    // Three regimes:
    //   banking to chase laterally  → half TRIM_THROTTLE (avoid overspeed in turn)
    //   diving (nav_pitch_cd < -500) → cut throttle further to avoid overspeed
    //   straight/deadband           → full TRIM_THROTTLE
    const bool diving = (plane.nav_pitch_cd < -500);  // nose down > 5 deg
    float throttle = ex_in_deadband
        ? plane.aparm.throttle_cruise.get()
        : plane.aparm.throttle_cruise.get() * 0.5f;
    if (diving) {
        throttle *= 0.5f;   // halve again when pitched into a dive
    }
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, throttle);

}
