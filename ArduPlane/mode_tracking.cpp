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

  Terminal: activates when (current_alt_msl - TRK_TGT_ALT) <= TRK_TERM_ALT.
            TRK_TGT_ALT / TRK_TGT_LAT / TRK_TGT_LON store the target's MSL
            altitude (m) and position (decimal degrees).
*/



// ── _enter ────────────────────────────────────────────────────────────────────
bool ModeTracking::_enter()
{
    _errorx_rad     = 0.0f;
    _errory_rad     = 0.0f;
    _last_msg_ms    = 0;
    _prev_update_ms = AP_HAL::millis();
    _was_timed_out      = false;
    _lock_stable_ms     = 0;   // 0 = not yet acquired
    _cruise_throttle    = plane.aparm.throttle_cruise.get();
    _terminal_entry_ms  = 0;   // 0 = not yet in terminal
    plane.g2.tracking_roll_pid.reset_I();
    plane.g2.tracking_roll_pid.reset_filter();
    plane.g2.tracking_pitch_pid.reset_I();
    plane.g2.tracking_pitch_pid.reset_filter();
    plane.g2.tracking_throt_pid.reset_I();
    plane.g2.tracking_throt_pid.reset_filter();

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
    plane.g2.tracking_throt_pid.reset_I();
    plane.g2.tracking_throt_pid.reset_filter();
    _cruise_throttle   = plane.aparm.throttle_cruise.get();
    plane.g2.tracking_throt_pid.reset_I();
    plane.g2.tracking_throt_pid.reset_filter();
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

    // Deadband is needed in both branches.
    const float deadband_rad = plane.g2.tracking_deadband_deg.get() * (M_PI / 180.0f);

    // in_terminal — hoisted so both PID and throttle branches can use it.
    // Activates when the drone is within TRK_TERM_ALT metres above TRK_TGT_ALT (MSL).
    const float term_alt        = plane.g2.tracking_term_alt.get();
    const float current_alt_msl = plane.current_loc.alt * 0.01f;   // cm → m MSL
    const float target_alt_msl  = plane.g2.tracking_target_alt_msl.get();
    const bool  in_terminal     = (term_alt > 0.0f &&
                                   (current_alt_msl - target_alt_msl) <= term_alt);

    // Settle ramp — computed here so throttle can also use it.
    // During timeout _lock_stable_ms is cleared, so ramp stays 0 until re-acquisition.
    const float settle_s = plane.g2.tracking_settle_s.get();
    float ramp = 1.0f;
    if (!timed_out && settle_s > 0.0f && _lock_stable_ms > 0) {
        const float elapsed = constrain_float((now_ms - _lock_stable_ms) * 1e-3f,
                                              0.0f, settle_s);
        ramp = elapsed / settle_s;
    } else if (timed_out) {
        ramp = 0.0f;
    }

    if (timed_out) {
        if (!_was_timed_out) {
            // Flush integrators on first timeout cycle to prevent wind-up.
            plane.g2.tracking_roll_pid.reset_I();
            plane.g2.tracking_roll_pid.reset_filter();
            plane.g2.tracking_pitch_pid.reset_I();
            plane.g2.tracking_pitch_pid.reset_filter();
        }

        // No signal — hold wings level.
        plane.nav_roll_cd  = 0;
        plane.nav_pitch_cd = 0;

        _was_timed_out  = true;
        _lock_stable_ms = 0;   // force re-settle when signal returns
    } else {
        if (_was_timed_out) {
            // Signal just returned — start settle timer now.
            _lock_stable_ms = now_ms;
        } else if (_lock_stable_ms == 0) {
            // First acquisition after mode entry.
            _lock_stable_ms = now_ms;
        }
        _was_timed_out = false;

        // ── Roll PID ─────────────────────────────────────────────────────────
        // errorx > 0 → target is to the right → roll right (positive bank).
        // Reset I when inside deadband to prevent integrator wind-up.
        const float ex = fabsf(_errorx_rad) > deadband_rad ? _errorx_rad : 0.0f;
        if (is_zero(ex)) {
            plane.g2.tracking_roll_pid.reset_I();
            // In deadband: use gyro roll rate to actively damp any rolling
            // motion and drive toward wings-level.  Each deg/s of roll rate
            // commands an opposing 50 cdeg (0.5 deg) of bank target so the
            // attitude controller fights the rotation rather than coasting.
            const float roll_rate_dps = degrees(ahrs.get_gyro().x);
            const float damp_cd       = -(roll_rate_dps * 50.0f) * ramp;
            plane.nav_roll_cd = constrain_int32((int32_t)damp_cd,
                                                -plane.roll_limit_cd,
                                                 plane.roll_limit_cd);
        } else {
            const float roll_cd = plane.g2.tracking_roll_pid.update_all(degrees(ex), 0.0f, dt_s)
                                  * ramp;
            plane.nav_roll_cd   = constrain_int32((int32_t)roll_cd,
                                                  -plane.roll_limit_cd,
                                                   plane.roll_limit_cd);
        }

        // ── Pitch PID ────────────────────────────────────────────────────────
        // errory > 0 → target is above → pitch up (positive setpoint).
        // TRK_PITCH_OFFSET: constant cruise bias (converted to rad).
        // TRK_TERM_PTCH:    additional pitch-down added in terminal phase only,
        //                   to counter the nose-up moment from full throttle.
        float pitch_offset_deg = plane.g2.tracking_pitch_offset.get();
        if (in_terminal) {
            pitch_offset_deg += plane.g2.tracking_term_pitch.get();  // add → larger offset → more nose-down
        }
        const float pitch_offset_rad = pitch_offset_deg * (M_PI / 180.0f);
        const float ey_raw = fabsf(_errory_rad) > deadband_rad ? _errory_rad : 0.0f;
        const float ey     = ey_raw - pitch_offset_rad;
        if (is_zero(ey_raw)) {
            plane.g2.tracking_pitch_pid.reset_I();
        }
        const float pitch_cd = plane.g2.tracking_pitch_pid.update_all(degrees(ey), 0.0f, dt_s)
                               * ramp;
        plane.nav_pitch_cd   = constrain_int32((int32_t)pitch_cd,
                                               (int32_t)(plane.pitch_limit_min * 100),
                                               plane.aparm.pitch_limit_max.get() * 100);
    }

    plane.update_load_factor();

    // ── Throttle ─────────────────────────────────────────────────────────────
    // Outside terminal: constant TRIM_THROTTLE, PID reset.
    // In terminal: PID drives throttle based on pitch error.
    //   A separate terminal ramp (0→1 over TRK_SETTLE_S) smooths the step
    //   when first entering terminal phase so there is no abrupt throttle cut.
    {
        const float cruise = plane.aparm.throttle_cruise.get();
        float throttle;
        const float nav_pitch_rad = plane.nav_pitch_cd * 0.01f * (M_PI / 180.0f);
        const float pitch_err     = ahrs.get_pitch() - nav_pitch_rad;
      

        if (in_terminal) {
            // Track first entry into terminal phase.
            if (_terminal_entry_ms == 0) {
                _terminal_entry_ms = now_ms;
                plane.g2.tracking_throt_pid.reset_I();
                plane.g2.tracking_throt_pid.reset_filter();
            }
            // Ramp 0→1 over settle_s seconds from terminal entry.
            float term_ramp = 1.0f;
            if (settle_s > 0.0f) {
                const float elapsed = constrain_float(
                    (now_ms - _terminal_entry_ms) * 1e-3f, 0.0f, settle_s);
                term_ramp = elapsed / settle_s;
            }
            const float pid_out       = plane.g2.tracking_throt_pid.update_all(0.0f, pitch_err, dt_s)
                                        * ramp * term_ramp;
          
            throttle = constrain_float(cruise + pid_out, cruise/3, cruise);
        } else {
            _terminal_entry_ms = 0;   // reset so ramp restarts next time
            
           const float pid_out       = plane.g2.tracking_throt_pid.update_all(0.0f, pitch_err, dt_s)
                                        * ramp;
          
            throttle = constrain_float(cruise + pid_out, cruise/3, cruise);
        }

        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, throttle);
    }

}
