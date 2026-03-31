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
    _was_timed_out   = false;
    _lock_stable_ms  = 0;   // 0 = not yet acquired
    _est_errorx_rad  = 0.0f;
    _est_errory_rad  = 0.0f;
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

    // Deadband is needed in both branches.
    const float deadband_rad = plane.g2.tracking_deadband_deg.get() * (M_PI / 180.0f);

    if (timed_out) {
        if (!_was_timed_out) {
            // First cycle of timeout: snapshot last known error for dead-reckoning
            // and flush integrators to prevent wind-up and stale derivative carry-over.
            _est_errorx_rad = _errorx_rad;
            _est_errory_rad = _errory_rad;
            plane.g2.tracking_roll_pid.reset_I();
            plane.g2.tracking_roll_pid.reset_filter();
            plane.g2.tracking_pitch_pid.reset_I();
            plane.g2.tracking_pitch_pid.reset_filter();
        }

        // Dead-reckoning: integrate body rates to propagate the estimated
        // target position in the camera frame.
        // Aircraft roll right → camera tilts right → target drifts left  → errorx decreases.
        // Aircraft pitch up   → camera tilts up   → target drifts down  → errory decreases.
        _est_errorx_rad -= ahrs.get_gyro().x * dt_s;
        _est_errory_rad -= ahrs.get_gyro().y * dt_s;

        // Exponential decay toward zero (τ ≈ 2 s) so the estimate gracefully
        // returns to wings-level if the signal is lost for an extended period.
        const float decay = expf(-dt_s * 0.5f);
        _est_errorx_rad *= decay;
        _est_errory_rad *= decay;

        // Clamp to ±30 deg equivalent.
        constexpr float MAX_EST_RAD = 30.0f * (M_PI / 180.0f);
        _est_errorx_rad = constrain_float(_est_errorx_rad, -MAX_EST_RAD,  MAX_EST_RAD);
        _est_errory_rad = constrain_float(_est_errory_rad, -MAX_EST_RAD,  MAX_EST_RAD);

        // Run PIDs on estimated error so the aircraft continues steering toward
        // the last-known target position rather than simply wings-levelling.
        const float est_ex = fabsf(_est_errorx_rad) > deadband_rad ? _est_errorx_rad : 0.0f;
        ex_in_deadband = is_zero(est_ex);
        if (is_zero(est_ex)) {
            plane.g2.tracking_roll_pid.reset_I();
            const float roll_rate_dps = degrees(ahrs.get_gyro().x);
            const float damp_cd       = -(roll_rate_dps * 50.0f);
            plane.nav_roll_cd = constrain_int32((int32_t)damp_cd,
                                                -plane.roll_limit_cd,
                                                 plane.roll_limit_cd);
        } else {
            const float roll_cd = plane.g2.tracking_roll_pid.update_all(degrees(est_ex), 0.0f, dt_s);
            plane.nav_roll_cd   = constrain_int32((int32_t)roll_cd,
                                                  -plane.roll_limit_cd,
                                                   plane.roll_limit_cd);
        }

        const float est_ey = fabsf(_est_errory_rad) > deadband_rad ? _est_errory_rad : 0.0f;
        if (is_zero(est_ey)) {
            plane.g2.tracking_pitch_pid.reset_I();
        }
        const float est_pitch_cd = plane.g2.tracking_pitch_pid.update_all(degrees(est_ey), 0.0f, dt_s);
        plane.nav_pitch_cd = constrain_int32((int32_t)est_pitch_cd,
                                             (int32_t)(plane.pitch_limit_min * 100),
                                             plane.aparm.pitch_limit_max.get() * 100);

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

        // ── Settle ramp ───────────────────────────────────────────────────────
        // Scale PID outputs from 0 → 1 over TRK_SETTLE_S seconds after each
        // acquisition so a large initial error does not cause an abrupt
        // nose-up or banking jerk.
        const float settle_s   = plane.g2.tracking_settle_s.get();
        float ramp = 1.0f;
        if (settle_s > 0.0f && _lock_stable_ms > 0) {
            const float elapsed = constrain_float((now_ms - _lock_stable_ms) * 1e-3f,
                                                  0.0f, settle_s);
            ramp = elapsed / settle_s;
        }

        // ── Roll PID ─────────────────────────────────────────────────────────
        // errorx > 0 → target is to the right → roll right (positive bank).
        // Reset I when inside deadband to prevent integrator wind-up.
        const float ex = fabsf(_errorx_rad) > deadband_rad ? _errorx_rad : 0.0f;
        ex_in_deadband = is_zero(ex);
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
        const float term_alt2 = plane.g2.tracking_term_alt.get();
        const float agl_m2    = plane.current_loc.alt * 0.01f
                                - plane.home.alt       * 0.01f;
        const bool  in_terminal = (term_alt2 > 0.0f && agl_m2 <= term_alt2);

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
    // Regimes evaluated in priority order:
    //
    // 1. Terminal phase (AGL < TRK_TERM_ALT and TRK_TERM_ALT > 0):
    //      Full throttle — maximise impact speed.
    //
    // 2. Settle window (first TRK_SETTLE_S seconds after entry/re-acquisition):
    //      Cruise throttle — keep the aircraft calm while attitude stabilises.
    //
    // 3. Approach with airspeed P-controller (TRK_APP_SPD > 0):
    //      Reduce target 20% while banking to avoid turn overspeed.
    //      Clamped to [10, 90]%.
    //
    // 4. Open-loop fallback (TRK_APP_SPD == 0):
    //      Banking → half TRIM_THROTTLE, straight → full TRIM_THROTTLE.

    const float term_alt = plane.g2.tracking_term_alt.get();
    const float agl_m    = plane.current_loc.alt * 0.01f
                           - plane.home.alt       * 0.01f;
    const bool  terminal = (term_alt > 0.0f && agl_m <= term_alt);

    // Settle window: hold cruise throttle for TRK_SETTLE_S seconds after
    // mode entry or re-acquisition so the aircraft stabilises before the
    // airspeed controller takes over.
    const uint32_t settle_ms  = (uint32_t)(plane.g2.tracking_settle_s.get() * 1000.0f);
    const bool     settling   = (_lock_stable_ms == 0) ||
                                (now_ms - _lock_stable_ms < settle_ms);

    float throttle;
    if (settling || terminal) {
        // ── Regime 2: settle — cruise throttle ───────────────────────────
        // Keep a calm, predictable throttle while the attitude loop is
        // still stabilising after entry or re-acquisition.
        throttle = plane.aparm.throttle_cruise.get();

    } else {
        const float app_spd = plane.g2.tracking_app_spd.get();
        float airspeed_ms   = 0.0f;
        const bool have_spd = ahrs.airspeed_EAS(airspeed_ms);

        if (app_spd > 0.0f && have_spd) {
            // ── Regime 3: airspeed P-controller ──────────────────────────
            // Reduce target speed 20% while banking to avoid turn overspeed.
            const float target_ms = ex_in_deadband ? app_spd : app_spd * 0.8f;
            const float spd_err   = target_ms - airspeed_ms;
            // Gain: 3 %throttle per m/s error.
            throttle = plane.aparm.throttle_cruise.get() + 3.0f * spd_err;
            throttle = constrain_float(throttle, 10.0f, 90.0f);

        } else {
            // ── Regime 4: open-loop fallback ─────────────────────────────
            throttle = ex_in_deadband
                ? plane.aparm.throttle_cruise.get()
                : plane.aparm.throttle_cruise.get() * 0.5f;
        }
    }
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, throttle);

}
