// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LauncherConstants;

public class Launcher extends SubsystemBase
{
    private final TalonFX indexerMotor = new TalonFX(LauncherConstants.INDEXER_MOTOR_CHANNEL,
        LauncherConstants.INDEXER_MOTOR_BUS);
    private final TalonFX leftShooterMotor = new TalonFX(LauncherConstants.LEFT_SHOOTER_MOTOR_CHANNEL,
        LauncherConstants.INDEXER_MOTOR_BUS);
    private final TalonFX rightShooterMotor = new TalonFX(LauncherConstants.RIGHT_SHOOTER_MOTOR_CHANNEL,
        LauncherConstants.INDEXER_MOTOR_BUS);

    /** Creates a new Launcher. */
    public Launcher()
    {
        var configs = new TalonFXConfiguration();
        configs.MotorOutput.Inverted = LauncherConstants.INDEXER_MOTOR_INVERTED_VALUE;
        indexerMotor.getConfigurator().apply(new TalonFXConfiguration());
        indexerMotor.getConfigurator().apply(configs);

        configs.MotorOutput.Inverted = LauncherConstants.LEFT_SHOOTER_MOTOR_INVERTED_VALUE;
        var slot0Configs = new Slot0Configs();
        slot0Configs.kS = 0.1; // Add 0.1 V output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kP = 0.11; // An error of 1 rps results in 0.11 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0; // no output for error derivative
        leftShooterMotor.getConfigurator().apply(new TalonFXConfiguration());
        leftShooterMotor.getConfigurator().apply(configs);
        leftShooterMotor.getConfigurator().apply(slot0Configs);

        configs.MotorOutput.Inverted = LauncherConstants.RIGHT_SHOOTER_MOTOR_INVERTED_VALUE;
        rightShooterMotor.getConfigurator().apply(new TalonFXConfiguration());
        rightShooterMotor.getConfigurator().apply(configs);
        rightShooterMotor
            .setControl(new Follower(LauncherConstants.LEFT_SHOOTER_MOTOR_CHANNEL, MotorAlignmentValue.Opposed));
    }

    public void feed()
    {
        // Confirm shooter motors are at speed, then
        // Run the upper indexer motor to move fuel to the shooter
    }

    public void clear()
    {
        // Clear the launcher by running the upper indexer motor in reverse
    }

    public void spinUpShooters(double speed)
    {
        // Set left and right shooter motors to a specific speed in RPM
    }

    public void idleShooters()
    {
        // Set left and right shooter motors to a low "idle" speed to keep shooter spinning
    }

    public void stopIndexer()
    {
        // Stop upper indexer motor
    }

    public void stopShooters()
    {
        // Stop left and right shooter motors
    }

    // Stops all 3 motors in launcher
    public void Stop()
    {
        // Stop the upper indexer motor
        stopIndexer();
        // Stop lefts and right shooter motors
        stopShooters();
    }

    @Override
    public void periodic()
    {
        // This method will be called once per scheduler run
    }
}
