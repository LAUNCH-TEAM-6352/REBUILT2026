// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants.DashboardConstants;

public class Launcher extends SubsystemBase
{
    private final TalonFX indexerMotor = new TalonFX(LauncherConstants.INDEXER_MOTOR_CHANNEL,
        LauncherConstants.INDEXER_MOTOR_BUS);
    private final TalonFX leftShooterMotor = new TalonFX(LauncherConstants.LEFT_SHOOTER_MOTOR_CHANNEL,
        LauncherConstants.INDEXER_MOTOR_BUS);
    private final TalonFX rightShooterMotor = new TalonFX(LauncherConstants.RIGHT_SHOOTER_MOTOR_CHANNEL,
        LauncherConstants.INDEXER_MOTOR_BUS);

    private final VelocityVoltage velocityVoltage = new VelocityVoltage(0);

    double targetVelocity;
    double targetTolerance;
    boolean atTargetVelocity;

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
            .setControl(new Follower(leftShooterMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    private void setIndexerSpeed(double speed)
    {
        indexerMotor.set(speed);
    }

    public void feed()
    {
        setIndexerSpeed(
            SmartDashboard.getNumber(DashboardConstants.LAUNCHER_FEED_KEY, LauncherConstants.FEED_SPEED));
    }

    public void clear()
    {
        setIndexerSpeed(
            SmartDashboard.getNumber(DashboardConstants.LAUNCHER_CLEAR_KEY, LauncherConstants.CLEAR_SPEED));
    }

    // Set shooter velocity in RPM
    private void setShooterVelocity(double velocity)
    {
        leftShooterMotor.setControl(velocityVoltage.withVelocity(velocity / 60));
        targetVelocity = velocity / 60;
    }

    public void spinUpShooters()
    {
        setShooterVelocity(
            SmartDashboard.getNumber(DashboardConstants.LAUNCHER_SHOOTING_KEY,
                LauncherConstants.SHOOTING_VELOCITY_RPM));
    }

    public void idleShooters()
    {
        setShooterVelocity(
            SmartDashboard.getNumber(DashboardConstants.LAUNCHER_IDLE_KEY, LauncherConstants.IDLE_VELOCITY_RPM));
    }

    public void stopIndexer()
    {
        indexerMotor.stopMotor();
    }

    public void stopShooters()
    {
        leftShooterMotor.stopMotor();
    }

    // Stops all 3 motors in launcher
    public void Stop()
    {
        stopIndexer();
        stopShooters();
    }

    @Override
    public void periodic()
    {
        double currentVelocity = (leftShooterMotor.getVelocity().getValueAsDouble()) / 60;
        if ((Math.abs(currentVelocity - targetVelocity) < targetTolerance)
            && Math.abs(currentVelocity - targetVelocity) < targetTolerance)
        {
            atTargetVelocity = true;
        }
        else
        {
            atTargetVelocity = false;
        }
    }
}
