//! Stationary attention, driven by timestamped camera-relative bearings.
//! Detection stays in the host. This module owns no camera, clock or motor.

use serde::Deserialize;

use crate::types::Command;

/// Explicit prototype tuning, supplied by the host (see `deploy/po.json`).
#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Config {
    pub mode: String,
    pub twist_rad: f32,
    pub pitch_rad: f32,
    pub roll_rad: f32,
    pub rate_rad_s: f32,
    pub deadband_rad: f32,
    pub confidence_min: f32,
    pub face_timeout_s: f32,
    pub lost_hold_s: f32,
    pub dwell_s: f32,
    pub scan_interval_s: f32,
}

#[derive(Clone, Copy, Debug)]
pub struct Face {
    /// Camera-relative angle, radians: positive left and up, respectively.
    pub azimuth: f32,
    pub elevation: f32,
    pub confidence: f32,
    /// Exposure time in the controller's monotonic microsecond time domain.
    pub at: u64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Phase {
    Scanning,
    Tracking,
    Holding,
    Returning,
}

pub struct Passive {
    pub config: Config,
    face: Option<Face>,
    last_sample: Option<u64>,
    processed: Option<u64>,
    last_tick: u64,
    next_move: u64,
    scan_at: u64,
    scan_side: f32,
    angles: [f32; 3],
    goal: [f32; 3],
    phase: Phase,
}

fn micros(s: f32) -> u64 {
    (f64::from(s) * 1e6) as u64
}

impl Config {
    pub fn validate(&self) -> Result<(), String> {
        let positive = [self.twist_rad, self.pitch_rad, self.roll_rad, self.rate_rad_s,
            self.face_timeout_s, self.lost_hold_s, self.dwell_s, self.scan_interval_s];
        if self.mode.is_empty() || positive.iter().any(|v| !v.is_finite() || *v <= 0.0)
            || positive.iter().any(|v| *v > 60.0)
            || !self.deadband_rad.is_finite() || self.deadband_rad < 0.0
            || self.deadband_rad >= self.twist_rad.min(self.pitch_rad)
            || !self.confidence_min.is_finite() || !(0.0..=1.0).contains(&self.confidence_min)
        {
            return Err("invalid passive tuning: use finite, positive bounds and durations".into());
        }
        Ok(())
    }
}

impl Passive {
    pub fn new(config: Config, now: u64) -> Result<Self, String> {
        config.validate()?;
        Ok(Self { config, face: None, last_sample: None, processed: None,
            last_tick: now, next_move: now, scan_at: now, scan_side: 1.0,
            angles: [0.0; 3], goal: [0.0; 3], phase: Phase::Scanning })
    }

    /// Invalid/old frames do not refresh attention or consume a newer frame.
    pub fn observe(&mut self, face: Face, received_at: u64) -> bool {
        if !face.azimuth.is_finite() || !face.elevation.is_finite()
            || face.azimuth.abs() > std::f32::consts::PI
            || face.elevation.abs() > std::f32::consts::FRAC_PI_2
            || !face.confidence.is_finite() || !(0.0..=1.0).contains(&face.confidence)
            || face.at > received_at
            || received_at - face.at > micros(self.config.face_timeout_s)
            || self.last_sample.is_some_and(|last| face.at <= last)
        {
            return false;
        }
        self.last_sample = Some(face.at);
        if face.confidence < self.config.confidence_min {
            return false;
        }
        self.face = Some(face);
        true
    }

    pub fn phase(&self) -> Phase { self.phase }

    /// Preserve the contract's neutral height. Never request a walking step.
    /// Bearings are residual errors, so apply each accepted exposure at most once.
    pub fn command(&mut self, rest: Command, now: u64) -> Command {
        let dt = now.saturating_sub(self.last_tick) as f32 * 1e-6;
        self.last_tick = now;
        if let Some(face) = self.face.filter(|f| {
            now >= f.at && now - f.at <= micros(self.config.face_timeout_s)
        }) {
            self.phase = Phase::Tracking;
            if now >= self.next_move && self.processed != Some(face.at) {
                if self.processed.is_none() {
                    // One curious tilt on acquisition, not a repeated rocking loop.
                    self.goal[2] = self.config.roll_rad * self.scan_side;
                    self.scan_side = -self.scan_side;
                }
                self.processed = Some(face.at);
                if face.azimuth.abs() > self.config.deadband_rad {
                    self.goal[0] = (self.angles[0] + face.azimuth)
                        .clamp(-self.config.twist_rad, self.config.twist_rad);
                }
                if face.elevation.abs() > self.config.deadband_rad {
                    // Positive policy pitch is nose DOWN. A face above is negative pitch.
                    self.goal[1] = (self.angles[1] - face.elevation)
                        .clamp(-self.config.pitch_rad, self.config.pitch_rad);
                }
                self.next_move = now.saturating_add(micros(self.config.dwell_s));
            }
        } else if self.face.is_some_and(|f| {
            now >= f.at && now - f.at <= micros(self.config.face_timeout_s + self.config.lost_hold_s)
        }) {
            self.phase = Phase::Holding;
        } else {
            if self.face.take().is_some() {
                self.processed = None;
                self.goal = [0.0; 3];
                self.scan_at = now.saturating_add(micros(self.config.scan_interval_s));
            }
            if now >= self.scan_at {
                self.goal = [self.scan_side * self.config.twist_rad, 0.0, 0.0];
                self.scan_side = -self.scan_side;
                self.scan_at = now.saturating_add(micros(self.config.scan_interval_s));
                self.phase = Phase::Scanning;
            } else if self.goal == [0.0; 3] {
                self.phase = Phase::Returning;
            } else {
                self.phase = Phase::Scanning;
            }
        }
        // Cap catch-up after a stalled host; do not jump through a missed second.
        let step = self.config.rate_rad_s * dt.min(0.05);
        for (angle, goal) in self.angles.iter_mut().zip(self.goal) {
            *angle += (goal - *angle).clamp(-step, step);
        }
        Command { base_twist: self.angles[0], base_pitch: self.angles[1],
            base_roll: self.angles[2], lin_vel_x: 0.0, lin_vel_y: 0.0, yaw_rate: 0.0,
            ..rest }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn config() -> Config {
        serde_json::from_str(include_str!("../../po.json")).unwrap()
    }

    #[test]
    fn face_above_looks_up_without_walking_or_zeroing_height() {
        let mut p = Passive::new(config(), 0).unwrap();
        assert!(p.observe(Face { azimuth: 0.2, elevation: 0.3, confidence: 1.0, at: 0 }, 0));
        let rest = Command { height: 0.10647, ..Default::default() };
        p.command(rest, 0);
        let c = p.command(rest, 50_000);
        assert!(c.base_pitch < 0.0 && c.base_twist > 0.0);
        assert_eq!((c.lin_vel_x, c.lin_vel_y, c.yaw_rate), (0.0, 0.0, 0.0));
        assert_eq!(c.height, rest.height);
        assert!(c.base_pitch.abs() <= config().rate_rad_s * 0.05 + 1e-6);
    }

    #[test]
    fn invalid_future_reordered_and_stale_frames_cannot_refresh_attention() {
        let mut p = Passive::new(config(), 0).unwrap();
        let f = Face { azimuth: 0.1, elevation: 0.2, confidence: 0.9, at: 100 };
        assert!(p.observe(f, 100));
        assert!(!p.observe(f, 200));
        assert!(!p.observe(Face { at: 201, ..f }, 200));
        assert!(!p.observe(Face { azimuth: f32::NAN, at: 200, ..f }, 200));
        assert!(!p.observe(Face { at: 200, ..f }, 2_000_000));
        p.command(Command::default(), 100);
        p.command(Command::default(), 600_000);
        assert_eq!(p.phase(), Phase::Holding);
        p.command(Command::default(), 2_000_000);
        assert_eq!(p.phase(), Phase::Returning);
    }

    #[test]
    fn sustained_error_stays_bounded_and_lost_target_returns() {
        let cfg = config();
        let mut p = Passive::new(cfg.clone(), 0).unwrap();
        let mut c = Command::default();
        for i in 0..4000 {
            let now = i * 10_000;
            if i < 2000 { p.observe(Face {
                azimuth: 1.0, elevation: 1.0, confidence: 1.0, at: now,
            }, now); }
            c = p.command(Command::default(), now);
            assert!(c.base_twist.abs() <= cfg.twist_rad + 1e-6);
            assert!(c.base_pitch.abs() <= cfg.pitch_rad + 1e-6);
            assert!(c.base_roll.abs() <= cfg.roll_rad + 1e-6);
            if i == 1000 { assert!(c.base_pitch < -0.1); }
        }
        assert_eq!(c.base_pitch, 0.0);
    }
}
