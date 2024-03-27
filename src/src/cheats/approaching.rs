use std::time::Duration;

use hs_hackathon::car::{Angle, MotorSocket, Velocity, WheelOrientation};
use hs_hackathon::drone::Camera;
use hs_hackathon::prelude::eyre;

use crate::cheats::positioning::{distance, Position};

use super::TeamColors;

/// A hint given to you by the approach cheat
pub enum Hint {
    /// Indicates that we are not on track anymore and we should repair the orientation
    OrientationIsOff(Angle),
    /// Indicates that the target was hit and we are in reasonable proximity
    TargetWasHit,
}

/// This approaches the target with your car
///
/// If the target is hit or a direction change is identified
/// this gives you a hint on what to do next
pub async fn auto(
    colors: &TeamColors,
    drone: &mut Camera,
    motor: &mut MotorSocket,
    wheels: &mut WheelOrientation,
) -> eyre::Result<Hint> {
    // todo: set a sane default for determining whether we are
    // "on" the target
    const SUCCESS_THRESHOLD: u32 = 100;
    const APPROACHING_DURATION: Duration = Duration::from_secs(2);

    'approaching: loop {
        let (precar, pretarget) = super::internal::infer(colors, drone).await?;
        let pre = distance(&precar, &pretarget);

        wheels.set(Angle::straight()).await?;
        motor
            .move_for(Velocity::forward(), APPROACHING_DURATION)
            .await?;

        let (currentcar, currenttarget) = super::internal::infer(colors, drone).await?;
        let current = distance(&currentcar, &currenttarget);

        // 1. if current is in a success threshold difference
        if current <= SUCCESS_THRESHOLD {
            return Ok(Hint::TargetWasHit);
        }

        // 2. if we were closer before approaching or didnt move, calibrate
        if pre <= current {
            let car_pos = Position::from(currentcar.to_owned());
            let targ_pos = Position::from(currenttarget.to_owned());
            let angle = car_pos.angle(&targ_pos);
            let angle = if angle > 0.0 {
                Angle::right()
            } else {
                Angle::left()
            };
            return Ok(Hint::OrientationIsOff(angle));
        }

        // 3. continue with approaching
        continue 'approaching;
    }
}
