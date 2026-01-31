// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    public static class ClimberConstants
    {
        // All ClimberConstants/values are copied & renamed from REEFSCAPE Climber constants.
        public static final int WINCH_MOTOR_CHANNEL = 0;
        public static final double WINCH_MOTOR_SPEED = 0.6;
        public static final boolean IS_MOTOR_INVERTED = false;

        public static final double MAX_POSITION = 140;
        public static final double MIN_POSITON = 0;

        public static final int SERVO_CHANNEL = 0;

        public static final int RATCHET_ENGAGED_POSITION = 1500;
        public static final int RATCHET_RELEASED_POSITION = 1000;
    }

    public static class IntakeConstants
    {
        public static final int INTAKE_MOTOR_CHANNEL = 0;
        public static final int PIVOT_MOTOR_CHANNEL = 0;

        public static final boolean IS_INTAKE_MOTOR_INVERTED = false;
        public static final boolean IS_PIVOT_MOTOR_INVERTED = false;

        public static final double INTAKE_SPEED_RPM = 0;
        public static final double EJECT_SPEED_RPM = 0;

        public static final double STOW_POSITION = 0;
        public static final double DEPLOYED_POSITION = 0;
        public static final double PARTIALLY_DEPLOYED_POSITION = 0;
    }

    public static class OperatorConstants
    {
        public static final int kDriverControllerPort = 0;
    }
}
