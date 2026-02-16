// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import com.pathplanner.lib.config.PIDConstants;

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
        // All ClimberConstants/values are copied & renamed from REEFSCAPE Climber constants.
        public static final CANBus WINCH_MOTOR_BUS = Constants.CANIVORE_BUS;
        public static final int WINCH_MOTOR_CHANNEL = 0;
        public static final double WINCH_MOTOR_SPEED = 0.6;
        public static final boolean IS_MOTOR_INVERTED = false;

        public static final double MAX_POSITION = 140;
        public static final double MIN_POSITON = 0;

        public static final int SERVO_CHANNEL = 0;

        public static final int RATCHET_ENGAGED_POSITION = 1500;
        public static final int RATCHET_RELEASED_POSITION = 1000;
    }

    public static class HopperConstants
    {
        public static final CANBus INDEXER_MOTOR_BUS = Constants.CANIVORE_BUS;
        public static final int INDEXER_MOTOR_CHANNEL = 41;
        public static final InvertedValue INDEXER_MOTOR_INVERTED_VALUE = InvertedValue.CounterClockwise_Positive;

        public static final double FEED_SPEED = 0.25;
        public static final double CLEAR_SPEED = -0.25;
    }

    public static class IntakeConstants
    {
        public static final CANBus INTAKE_MOTOR_BUS = Constants.CANIVORE_BUS;
        public static final int INTAKE_MOTOR_CHANNEL = 45;
        public static final CANBus PIVOT_MOTOR_BUS = Constants.CANIVORE_BUS;
        public static final int PIVOT_MOTOR_CHANNEL = 46;
        public static final int PIVOT_ENCODER_CHANNEL = 47;

        public static final InvertedValue INTAKE_MOTOR_INVERTED_VALUE = InvertedValue.Clockwise_Positive;
        public static final InvertedValue PIVOT_MOTOR_INVERTED_VALUE = InvertedValue.CounterClockwise_Positive;

        public static final double INTAKE_SPEED = 0.5;
        public static final double EJECT_SPEED = -0.5;

        public static final double PIVOT_SPEED = 0.5;

        public static final double INTAKE_KP = 0.15;
        public static final double INTAKE_KI = 0.0;
        public static final double INTAKE_KD = 0.0;

        public static final double DEPLOY_TOLERANCE = 10;

        // These positions are in degrees, and represent the angle of the pivot motor when the intake is in each
        // position.
        public static final double STOW_POSITION = 0;
        public static final double DEPLOYED_POSITION = 130;
        public static final double PARTIALLY_DEPLOYED_POSITION = 65;
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

        public static final double SHOOTING_VELOCITY_RPM = 300;
        public static final double IDLE_VELOCITY_RPM = 100;
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
        public static final int kDriverControllerPort = 0;
        public static final int DRIVER_GAMEPAD_PORT = 0;
        public static final int CODRIVER_GAMEPAD_PORT = 1;
    }

    public static final class PathPlannerConstants
    {
        public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
        public static final PIDConstants ANGLE_PID = new PIDConstants(0.4, 0, 0.01);
    }

    public static final class DashboardConstants
    {
        public static final String HOPPER_INDEXER_FEED_KEY = "HopperFeed";
        public static final String HOPPER_INDEXER_CLEAR_KEY = "HopperClear";

        public static final String INTAKE_SPEED_KEY = "Intake";
        public static final String EJECT_SPEED_KEY = "Eject";
        public static final String PIVOT_SPEED_KEY = "Pivot";
        public static final String PIVOT_TOLERANCE_KEY = "PivotTol";
        public static final String DEPLOY_KEY = "Deploy";
        public static final String PARTIALLY_DEPLOY_KEY = "PartialDeploy";
        public static final String STOW_KEY = "Stow";

        public static final String LAUNCHER_SHOOTING_KEY = "LauncherShooting";
        public static final String LAUNCHER_IDLE_KEY = "LauncherIdle";
        public static final String LAUNCHER_FEED_KEY = "LauncherFeed";
        public static final String LAUNCHER_CLEAR_KEY = "LauncherClear";
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
