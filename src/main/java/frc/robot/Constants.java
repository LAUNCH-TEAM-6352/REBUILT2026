// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{
    public static final CANBus CANIVORE_BUS = new CANBus("skycan");

    public static class ClimberConstants
    {
        public static final CANBus WINCH_MOTOR_BUS = Constants.CANIVORE_BUS;
        public static final int WINCH_MOTOR_CHANNEL = 48;
        public static final double WINCH_MOTOR_SPEED = 0.6;
        public static final InvertedValue WINCH_MOTOR_INVERTED_VALUE = InvertedValue.Clockwise_Positive;

        public static final double WINCH_KP = 0.14;
        public static final double WINCH_KI = 0;
        public static final double WINCH_KD = 0;

        public static final double WINCH_MAX_FWD_SPEED = 0.1;
        public static final double WINCH_MAX_REV_SPEED = -0.4;
        public static final double WINCH_MAX_MAN_SPEED = 0.4;

        // These extended and climbed positions are WAGs based upon limited testing on 2026-02-21:
        public static final double STOWED_POSITION = 0;
        public static final double EXTENDED_POSITION = 55.3;
        public static final double CLIMBED_POSITION = 39;

        public static final double MIN_POSITION = STOWED_POSITION;
        public static final double MAX_POSITION = EXTENDED_POSITION;

        public static final int SERVO_CHANNEL = 0;

        public static final int RATCHET_ENGAGED_POSITION = 0;
        public static final int RATCHET_RELEASED_POSITION = 1;
    }

    public static class HopperConstants
    {
        public static final CANBus CONVEYOR_MOTOR_BUS = Constants.CANIVORE_BUS;
        public static final int CONVEYOR_MOTOR_CHANNEL = 41;
        public static final InvertedValue CONVEYOR_MOTOR_INVERTED_VALUE = InvertedValue.CounterClockwise_Positive;

        public static final double FEED_SPEED = 0.25;
        public static final double CLEAR_SPEED = -0.25;
    }

    public static class IntakeConstants
    {
        public static final CANBus INTAKE_MOTOR_BUS = Constants.CANIVORE_BUS;
        public static final int INTAKE_MOTOR_CHANNEL = 45;
        public static final CANBus PIVOT_MOTOR_BUS = Constants.CANIVORE_BUS;
        public static final int PIVOT_MOTOR_CHANNEL = 46;
        public static final CANBus PIVOT_ENCODER_BUS = Constants.CANIVORE_BUS;
        public static final int PIVOT_ENCODER_CHANNEL = 47;

        public static final InvertedValue INTAKE_MOTOR_INVERTED_VALUE = InvertedValue.Clockwise_Positive;
        public static final InvertedValue PIVOT_MOTOR_INVERTED_VALUE = InvertedValue.CounterClockwise_Positive;

        public static final SensorDirectionValue ENCODER_DIRECTION_VALUE = SensorDirectionValue.Clockwise_Positive;

        public static final double INTAKE_SPEED = 0.5;
        public static final double EJECT_SPEED = -0.5;

        public static final double PIVOT_KP = 0.8;
        public static final double PIVOT_KI = 0.0;
        public static final double PIVOT_KD = 0.0;

        public static final double PIVOT_MAX_FWD_SPEED = .6;
        public static final double PIVOT_MAX_REV_SPEED = -1;

        public static final double PIVOT_TOLERANCE = 10;

        // These positions are in degrees and represent the angle of the intake
        // relative to its starting/stowed position.
        public static final Angle STOWED_POSITION = Degrees.of(0);
        public static final Angle DEPLOYED_POSITION = Degrees.of(130);
        public static final Angle PARTIALLY_DEPLOYED_POSITION = Degrees.of(65);

        // These positions are in rotations and are used to set software limits on the pivot motor.
        public static final double MIN_POSITION = STOWED_POSITION.in(Rotations);
        public static final double MAX_POSITION = DEPLOYED_POSITION.in(Rotations);
    }

    public static class LauncherConstants
    {
        public static final CANBus INDEXER_MOTOR_BUS = Constants.CANIVORE_BUS;
        public static final int INDEXER_MOTOR_CHANNEL = 42;
        public static final CANBus LEFT_SHOOTER_MOTOR_BUS = Constants.CANIVORE_BUS;
        public static final int LEFT_SHOOTER_MOTOR_CHANNEL = 43;
        public static final CANBus RIGHT_SHOOTER_MOTOR_BUS = Constants.CANIVORE_BUS;
        public static final int RIGHT_SHOOTER_MOTOR_CHANNEL = 44;

        public static final InvertedValue INDEXER_MOTOR_INVERTED_VALUE = InvertedValue.Clockwise_Positive;
        public static final InvertedValue LEFT_SHOOTER_MOTOR_INVERTED_VALUE = InvertedValue.Clockwise_Positive;
        public static final InvertedValue RIGHT_SHOOTER_MOTOR_INVERTED_VALUE = InvertedValue.CounterClockwise_Positive;

        public static final double SHOOTING_VELOCITY_RPM = 3000;
        public static final double IDLE_VELOCITY_RPM = 1000;
        public static final double FEED_SPEED = 0.25;
        public static final double CLEAR_SPEED = -0.25;

        public static final double SHOOTER_KS = 0.1; // Add 0.1 V output to overcome static friction
        public static final double SHOOTER_KV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        public static final double SHOOTER_KP = 0.11; // An error of 1 rps results in 0.11 V output
        public static final double SHOOTER_KI = 0.0; // no output for integrated error
        public static final double SHOOTER_KD = 0.0; // no output for error derivative
    }

    public static class OperatorConstants
    {
        public static final int DRIVER_GAMEPAD_PORT = 0;
        public static final int CODRIVER_GAMEPAD_PORT = 1;
    }

    public static final class PathPlannerConstants
    {
        public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
        public static final PIDConstants ANGLE_PID = new PIDConstants(0.4, 0, 0.01);
        public static final double MAX_VELOCITY_MPS = 3.0;
        public static final double MAX_ACCELERATION_MPS_SQ = 4.0;
        public static final double MAX_ANGULAR_VELOCITY_RPS = Units.degreesToRadians(540);
        public static final double MAX_ANGULAR_ACCELERATION_RPS = Units.degreesToRadians(720);
    }

    public static final class DashboardConstants
    {
        public static final String CLIMBER_CLIMB_KEY = "Climb Position";
        public static final String CLIMBER_EXTEND_KEY = "Extended Position";
        public static final String CLIMBER_STOW_KEY = "Stowed Position";

        public static final String CONVEYOR_FEED_KEY = "HopperFeed";
        public static final String CONVEYOR_CLEAR_KEY = "HopperClear";

        public static final String INTAKE_SPEED_KEY = "IntakeSpd";
        public static final String EJECT_SPEED_KEY = "EjectSpd";
        public static final String PIVOT_TOLERANCE_KEY = "PivotTol";
        public static final String DEPLOYED_KEY = "DeployedPos";
        public static final String PARTIALLY_DEPLOYED_KEY = "PartialDeployedPos";
        public static final String STOWED_KEY = "StowedPos";

        public static final String LAUNCHER_SHOOTING_KEY = "LauncherShooting";
        public static final String LAUNCHER_IDLE_KEY = "LauncherIdle";
        public static final String LAUNCHER_FEED_KEY = "LauncherFeed";
        public static final String LAUNCHER_CLEAR_KEY = "LauncherClear";

        public static final String LIMELIGHT_THROTTLE_DISABLED_KEY = "LL Throttle Off";
        public static final String LIMELIGHT_THROTTLE_ENABLED_KEY = "LL Throttle On";
    }

    public static final class LimeLightConstants
    {
        public static final double LIMELIGHT_THROTTLE_DISABLED = 0.0;
        public static final double LIMELIGHT_THROTTLE_ENABLED = 200.0;
    }

    public static class TestConstants
    {
        public static final double BETWEEN_TIME_SECS = 1.0;
        public static final double INSTANT_BETWEEN_TIME_SECS = 3.0;

        public static final double SWERVE_MODULE_MOTOR_TIMEOUT_SECS = 5;
        public static final double SWERVE_MODULE_DRIVE_FORWARD_SPEED = 0.20;
        public static final double SWERVE_MODULE_DRIVE_REVERSE_SPEED = -0.20;
        public static final double SWERVE_MODULE_STEER_CCW_SPEED = 0.20;
        public static final double SWERVE_MODULE_STEER_CW_SPEED = -0.20;

        public static final String[] swerveModuleNames = {
                        // Swerve module names in the order in which they are indexed:
                        "FrontLeft",
                        "FrontRight",
                        "BackLeft",
                        "BackRight"
        };
    }

}
