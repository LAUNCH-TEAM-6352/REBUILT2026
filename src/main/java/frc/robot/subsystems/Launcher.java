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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants.DashboardConstants;

public class Launcher extends SubsystemBase
{
    private final TalonFX indexerMotor = new TalonFX(LauncherConstants.INDEXER_MOTOR_CHANNEL,
        LauncherConstants.INDEXER_MOTOR_BUS);
    private final TalonFX leftShooterMotor = new TalonFX(LauncherConstants.LEFT_SHOOTER_MOTOR_CHANNEL,
        LauncherConstants.LEFT_SHOOTER_MOTOR_BUS);
    private final TalonFX rightShooterMotor = new TalonFX(LauncherConstants.RIGHT_SHOOTER_MOTOR_CHANNEL,
        LauncherConstants.RIGHT_SHOOTER_MOTOR_BUS);

    private final VelocityVoltage velocityVoltage = new VelocityVoltage(0).withSlot(0);

    /** Creates a new Launcher. */
    public Launcher()
    {
        var configs = new TalonFXConfiguration();
        configs.MotorOutput.Inverted = LauncherConstants.INDEXER_MOTOR_INVERTED_VALUE;
        indexerMotor.getConfigurator().apply(configs);

        configs.MotorOutput.Inverted = LauncherConstants.LEFT_SHOOTER_MOTOR_INVERTED_VALUE;
        var slot0Configs = new Slot0Configs();
        slot0Configs.kS = LauncherConstants.SHOOTER_KS;
        slot0Configs.kV = LauncherConstants.SHOOTER_KV;
        slot0Configs.kP = LauncherConstants.SHOOTER_KP;
        slot0Configs.kI = LauncherConstants.SHOOTER_KI;
        slot0Configs.kD = LauncherConstants.SHOOTER_KD;
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

    // Intended for use with a press-and-hold binding
    public Command feedThenStopCommand()
    {
        return startEnd(this::feed, this::stopIndexer);
    }

    // Intended for use in auto (Only starts the indexer, does not stop automatically)
    public Command feedCommand()
    {
        return runOnce(this::feed);
    }

    public void feed()
    {
        setIndexerSpeed(
            SmartDashboard.getNumber(DashboardConstants.LAUNCHER_FEED_KEY, LauncherConstants.FEED_SPEED));
    }

    // Intended for use with a press-and-hold binding
    public Command clearThenStopCommand()
    {
        return startEnd(this::clear, this::stopIndexer);
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
    }

    public Command spinUpShootersCommand()
    {
        return runOnce(this::spinUpShooters);
    }

    public void spinUpShooters()
    {
        setShooterVelocity(
            SmartDashboard.getNumber(DashboardConstants.LAUNCHER_SHOOTING_KEY,
                LauncherConstants.SHOOTING_VELOCITY_RPM));
    }

    public Command idleShootersCommand()
    {
        return runOnce(this::idleShooters);
    }

    public void idleShooters()
    {
        setShooterVelocity(
            SmartDashboard.getNumber(DashboardConstants.LAUNCHER_IDLE_KEY, LauncherConstants.IDLE_VELOCITY_RPM));
    }

    public Command stopIndexerCommand()
    {
        return runOnce(this::stopShooters);
    }

    public Command stopShootersCommand()
    {
        return runOnce(this::stopShooters);
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
    public void stopAll()
    {
        stopIndexer();
        stopShooters();
    }

    @Override
    public void periodic()
    {
        // TODO: determine if shooter is at target velocity?
    }
}
