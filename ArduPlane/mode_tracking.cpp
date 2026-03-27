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
    Tunable via parameters TRAK_ROLL_P / _I / _D / _IMAX
    Default: P=200 cd/deg, I=10, D=5, imax=3000 cd

  Pitch control (PID on errory → nav_pitch_cd):
    Tunable via parameters TRAK_PTCH_P / _I / _D / _IMAX
    Default: P=100 cd/deg, I=500, D=0, imax=3000 cd

  Throttle: constant TRIM_THROTTLE percent.
  Timeout:  if no message within TRACKING_TIMEOUT_MS, both PIDs are
            reset and roll/pitch commands are zeroed (hold level until
            signal returns).
*/

// Deadband: ignore errors smaller than this (radians, ~0.6 deg)
static constexpr float    TRACKING_DEADBAND_RAD = 0.01f;

// Signal timeout
static constexpr uint32_t TRACKING_TIMEOUT_MS   = 1000;


// ── _enter ────────────────────────────────────────────────────────────────────
bool ModeTracking::_enter()
{
    _errorx_rad     = 0.0f;
    _errory_rad     = 0.0f;
    _last_msg_ms    = 0;
    _prev_update_ms = AP_HAL::millis();

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
    plane.g2.tracking_pitch_pid.reset_I();
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

    const bool timed_out = (_last_msg_ms == 0) ||
                           (now_ms - _last_msg_ms > TRACKING_TIMEOUT_MS);

    if (timed_out) {
        // No recent tracking signal: hold wings level, reset PIDs so there
        // is no integrator wind-up while the signal is absent.
        plane.g2.tracking_roll_pid.reset_I();
        plane.g2.tracking_pitch_pid.reset_I();
        plane.nav_roll_cd  = 0;
        plane.nav_pitch_cd = 0;
    } else {
        // ── Roll PID ─────────────────────────────────────────────────────────
        // errorx > 0 → target is to the right → roll right (positive bank).
        // Deadband applied before the PID to avoid integrator wind-up near zero.
        const float ex       = fabsf(_errorx_rad) > TRACKING_DEADBAND_RAD ? _errorx_rad : 0.0f;
        const float roll_cd  = plane.g2.tracking_roll_pid.update_error(degrees(ex), dt_s);
        plane.nav_roll_cd    = constrain_int32((int32_t)roll_cd,
                                               -plane.roll_limit_cd,
                                                plane.roll_limit_cd);

        // ── Pitch PID ────────────────────────────────────────────────────────
        // errory > 0 → target is above → pitch up (positive setpoint).
        const float ey       = fabsf(_errory_rad) > TRACKING_DEADBAND_RAD ? _errory_rad : 0.0f;
        const float pitch_cd = plane.g2.tracking_pitch_pid.update_error(degrees(ey), dt_s);
        plane.nav_pitch_cd   = constrain_int32((int32_t)pitch_cd,
                                               (int32_t)(plane.pitch_limit_min * 100),
                                               plane.aparm.pitch_limit_max.get() * 100);
    }

    plane.update_load_factor();

    // ── Constant throttle ────────────────────────────────────────────────────
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle,
                                    plane.aparm.throttle_cruise.get());
}
