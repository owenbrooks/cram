#[derive(Debug, Copy, Clone)]
pub enum RobotCommand {
    TurnLeft,
    TurnRight,
    Forward,
    Back,
    Stop,
}

#[derive(Debug, Copy, Clone)]
pub struct Pose {
    pub x: f32,
    pub y: f32,
    pub theta: f32,
}

#[derive(Debug, Copy, Clone)]
struct WheelVel {
    pub left: f32,
    pub right: f32,
}

#[derive(Debug, Copy, Clone)]
pub struct RobotState {
    pub pose: Pose,
    wheel_velocity: WheelVel,
}

pub struct Robot {
    pub state: RobotState,
    wheel_base: f32,
    pub command: RobotCommand,
}

const TURN_VEL: f32 = 0.1;
const LINEAR_VEL: f32 = 0.1;

impl Robot {
    pub fn new(wheel_base: f32) -> Self {
        Robot {
            wheel_base,
            state: RobotState {
                pose: Pose {
                    x: 0.,
                    y: 0.,
                    theta: 0.,
                },
                wheel_velocity: WheelVel {
                    left: 0.,
                    right: 0.,
                },
            },
            command: RobotCommand::Stop,
        }
    }
    pub fn set_command(&mut self, command: RobotCommand) {
        self.command = command;
    }
    pub fn step(&mut self, time_step: f32) {
        let wheel_vel = match self.command {
            RobotCommand::Forward => WheelVel {
                left: LINEAR_VEL,
                right: LINEAR_VEL,
            },
            RobotCommand::Back => WheelVel {
                left: -LINEAR_VEL,
                right: -LINEAR_VEL,
            },
            RobotCommand::TurnLeft => WheelVel {
                left: -TURN_VEL,
                right: TURN_VEL,
            },
            RobotCommand::TurnRight => WheelVel {
                left: TURN_VEL,
                right: -TURN_VEL,
            },
            RobotCommand::Stop => WheelVel {
                left: 0.,
                right: 0.,
            },
        };
        self.state.wheel_velocity = wheel_vel;

        let linear_vel = (wheel_vel.right + wheel_vel.left) / 2.0;
        let angular_vel = (wheel_vel.right - wheel_vel.left) / 2.0;

        let x_dot = linear_vel * self.state.pose.theta.cos();
        let y_dot = linear_vel * self.state.pose.theta.sin();
        let theta_dot = angular_vel / self.wheel_base;

        let new_state = RobotState {
            pose: Pose {
                x: self.state.pose.x + x_dot * time_step,
                y: self.state.pose.y + y_dot * time_step,
                theta: self.state.pose.theta + theta_dot * time_step,
            },
            wheel_velocity: wheel_vel,
        };
        self.state = new_state;
    }
}
