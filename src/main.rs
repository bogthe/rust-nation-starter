mod cheats;

use std::time::Duration;

use hs_hackathon::car::{Angle, MotorSocket, Velocity, WheelOrientation};
use hs_hackathon::drone::Camera;
use hs_hackathon::prelude::eyre;

use crate::cheats::positioning::distance;

use cheats::TeamColors;

use hs_hackathon::prelude::*;

use cheats::angles::Vector;
use cheats::approaching::Hint;
use cheats::positioning::Position;

const CAR: Color = Color::Red;
const TARGET: Color = Color::Blue;

#[allow(unused)]
struct MapState {
    car: Position,
    target: Position,
}

#[allow(unused)]
impl MapState {
    // How does this api look like?
    // - MapState::infer(drone) - gives me the supposed angle of if the car is going in the right
    // direction. Actually it would be nice if it would give me the car and target direction
    // internally;
    // - I move the car based on its state, and then I say what's the car orientation, the problem
    // on this, is that the movement, before and after is dependent on the car orientation
    // somehow?? To rephrase this, what I think I would need in this case is:
    //   - Always create a vector between current position and target position (desired travel
    //   vector), we can normalize it (probably SHOULD, but hey ho we'll figure it out).
    //   - Then we move in some direction that the wheels and the car are pointing in to;
    //   - Only then we can find the car orientation;
    //   - So I guess I don't fully get it at this point;
    // let state = MapState::infer();
    // let direction = state.car_orientation();
    // turn_wheels_for(delta(desired, direction), 2seconds);
    //  => Approaching, goes in front
    //  => Turns again
    //
    //  Disregard everything, I won't use this yet
    pub async fn infer(drone: &mut Camera) -> eyre::Result<Self> {
        let colors = TeamColors {
            car: CAR,
            target: TARGET,
        };
        let (car, target) = cheats::internal::infer(&colors, drone).await?;
        // No idea why I did this??
        Ok(Self {
            car: car.into(),
            target: target.into(),
        })
    }

    // This is going for a vector solution, not a pure angle one which I guess works the same as
    // the pure angle one, you get the angle between two vectors and then you correct based on that
    // angle
    async fn car_orientation(
        current: Position,
        drone: &mut Camera,
        motor: &mut MotorSocket,
        wheels: &mut WheelOrientation,
    ) -> eyre::Result<Vector> {
        const APPROACHING_DURATION: Duration = Duration::from_secs(2);
        let colors = TeamColors {
            car: CAR,
            target: TARGET,
        };

        wheels.set(Angle::straight()).await?;
        motor
            .move_for(Velocity::forward(), APPROACHING_DURATION)
            .await?;

        let (newcar, newtarget) = cheats::internal::infer(&colors, drone).await?;
        let current_distance = distance(&newcar, &newtarget);
        Ok((current, newcar.into()).into())
    }
}

#[derive(Debug)]
#[allow(unused)]
enum State {
    /// Turn the cars direction by doing consecutive front and back movements
    /// until the angle between the cars orientation and the target converges to be under
    /// a specified threshold
    Turning,
    /// Approach the car by doing incremental actions of approaching and measuring interleaved.
    /// So we approach the target a bit, measure if we decreased the distance, if yes repeat, if no
    /// then calibrate. We do this until we hit the target.
    Approaching,
    /// Simply idling on the target and identifying when the target moves away from our current
    /// position.
    Idle,
}

///             0
///       -45   |    45
///         \       /
///    
///   -90 -           – 90
///    
///         /       \
///      -135   |   135
///            180
pub async fn turning(
    colors: &TeamColors,
    drone: &mut Camera,
    motor: &mut MotorSocket,
    wheels: &mut WheelOrientation,
) -> eyre::Result<()> {
    // 1. Calculate the angle between current position and target
    // 2. Move the car in whatever direction it currently is
    // 3. Understand how much you need to rotate the wheels
    // # Proposition with angles (vecs come later):
    // Let's say angle was initially 45, and the vehicle moves down
    // The angle becomes 40, this means the delta is -5. This means we have to turn the
    // wheels to the left because we're moving on a negative angle.
    // By how much? I think it's easier to start with an angle of 0, car is right above
    // Interesting, I don't think this holds at 45 angle but let's go through some examples;
    //
    // Angle 0, and I move down at 180, distance increases, angle stays the same (unlikely) this
    // means turn left or right
    //
    // Angle 0 and I go right (at 90), now, the new angle is something like -5, wheels need to turn
    // left, I turn the wheel left something like MAX_WHEEL_TURN * (5 / 180) and move forward some
    // time. The minus was transformed in the "right or left" axis for the wheels
    //
    // Angle 0, I go left at -90, angle is something like 5, wheels need to turn right, same
    // calculation as above
    //
    // The problem with this example above is that the weels need to turn right quite a lot, not
    // just by 5 degrees. For example if I go 90 degrees in the wrong direction, I should correct
    // and go 90 degrees in the right direction. So I think the angle difference here should be
    // before and after my car position and before and after car target position. So, 90 degrees in
    // the wrong direction
    // TO THE EXAMPLES:
    // - Angle 0 (prec, pret)
    // - TAngle 5 (c,t) => means my car basically went left
    // - CAngle 90 (prec, c) => how bad my car went left, it could go left cause I did 90, but it
    // can also go left cause I did 135 or 180 degrees!
    // - So here we have the calculation CAngle / 180 * TAngleSign?
    //
    // Looks pretty straight forward so far. What can break?
    // - Stuck in a wall => no motion => go back X seconds, add more 'accent' to the turn (turn
    // left / right harder)
    // - TBD
    loop {
        const APPROACHING_DURATION: Duration = Duration::from_secs(1);
        const TURNING_DURATION: Duration = Duration::from_secs(1);

        let (precar, pretarget) = cheats::internal::infer(colors, drone).await?;
        let precarpos: Position = precar.into();
        let pretargetpos: Position = pretarget.into();
        // WARNING:cvvbiueujlt qqnected to my angle calculation
        let preangle = precarpos.angle(&pretargetpos.into());
        let pre = distance(&precar, &pretarget);
        if pre < 100 {
            return Ok(());
        }

        wheels.set(Angle::straight()).await?;
        motor
            .move_for(Velocity::forward(), APPROACHING_DURATION)
            .await?;

        let (car, target) = cheats::internal::infer(colors, drone).await?;
        let carpos: Position = car.into();
        let targetpos: Position = target.into();

        // Brought from description here
        // - Angle 0 (prec, pret)
        // - TAngle 5 (c,t) => means my car basically went left
        // - CAngle 90 (prec, c) => how bad my car went left, it could go left cause I did 90, but it
        // can also go left cause I did 135 or 180 degrees!
        // - So here we have the calculation CAngle / 180 * TAngleSign?
        let t_angle = carpos.angle(&targetpos.into());
        let c_angle = carpos.angle(&precarpos.into());
        let wheels_angle = if t_angle > 0.0 {
            c_angle / 180.0
        } else {
            c_angle / 180.0 * -1.0
        };

        let wheels_angle = wheels_angle as f32;
        wheels.set(wheels_angle.try_into().unwrap()).await?;
        motor
            .move_for(Velocity::forward(), TURNING_DURATION)
            .await?;

        // TODO do something to move backward based on distance travelled
        let current = distance(&car, &target);
        if current < 100 {
            return Ok(());
        }
    }
}

impl State {
    async fn execute(
        &mut self,
        drone: &mut Camera,
        motor: &mut MotorSocket,
        wheels: &mut WheelOrientation,
    ) -> eyre::Result<()> {
        match self {
            State::Turning => {
                turning(
                    &TeamColors {
                        car: CAR,
                        target: TARGET,
                    },
                    drone,
                    motor,
                    wheels,
                )
                .await?;
            }
            State::Approaching => {
                let hint = cheats::approaching::auto(
                    &TeamColors {
                        car: CAR,
                        target: TARGET,
                    },
                    drone,
                    motor,
                    wheels,
                )
                .await?;

                *self = match hint {
                    Hint::TargetWasHit => Self::Idle,
                    Hint::OrientationIsOff => Self::Turning,
                };
            }
            State::Idle => {
                cheats::idling::auto(
                    &TeamColors {
                        car: CAR,
                        target: TARGET,
                    },
                    drone,
                    motor,
                    wheels,
                )
                .await?;

                *self = Self::Turning;
            }
        }

        Ok(())
    }
}

#[hs_hackathon::main]
async fn main() -> eyre::Result<()> {
    let mut wheels = WheelOrientation::new().await?;
    let mut motor = MotorSocket::open().await?;
    let mut drone = Camera::connect().await?;

    let mut machine = State::Turning;

    loop {
        machine.execute(&mut drone, &mut motor, &mut wheels).await?;
        tracing::debug!("{:?}", machine);
    }
}
