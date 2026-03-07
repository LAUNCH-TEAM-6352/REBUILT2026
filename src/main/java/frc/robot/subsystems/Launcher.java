// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

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

    private final VelocityVoltage velocityVoltage = new VelocityVoltage(0).withSlot(0);

    private double targetVelocity = 0;
    private boolean atTargetVelocity = false;

    /** Creates a new Launcher. */
    public Launcher()
    {
        var configs = new TalonFXConfiguration();
        configs.MotorOutput.Inverted = LauncherConstants.INDEXER_MOTOR_INVERTED_VALUE;
        configs.Slot0.kS = LauncherConstants.INDEXER_KS;
        configs.Slot0.kV = LauncherConstants.INDEXER_KV;
        configs.Slot0.kP = LauncherConstants.INDEXER_KP;
        configs.Slot0.kI = LauncherConstants.INDEXER_KI;
        configs.Slot0.kD = LauncherConstants.INDEXER_KD;
        indexerMotor.getConfigurator().apply(configs);
        indexerMotor.getConfigurator().apply(configs.Slot0);
        indexerMotor.clearStickyFaults();

        configs.MotorOutput.Inverted = LauncherConstants.LEFT_SHOOTER_MOTOR_INVERTED_VALUE;
        var slot0Configs = new Slot0Configs();
        slot0Configs.kS = LauncherConstants.SHOOTER_KS;
        slot0Configs.kV = LauncherConstants.SHOOTER_KV;
        slot0Configs.kP = LauncherConstants.SHOOTER_KP;
        slot0Configs.kI = LauncherConstants.SHOOTER_KI;
        slot0Configs.kD = LauncherConstants.SHOOTER_KD;
        leftShooterMotor.getConfigurator().apply(configs);
        leftShooterMotor.getConfigurator().apply(slot0Configs);
        leftShooterMotor.clearStickyFaults();
    }

    private void setIndexerVelocity(double velocity)
    {
        indexerMotor.setControl(velocityVoltage.withVelocity(velocity / 60));

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
        setIndexerVelocity(
            SmartDashboard.getNumber(DashboardConstants.LAUNCHER_FEED_KEY, LauncherConstants.FEED_VELOCITY_RPM));
    }

    // Intended for use with a press-and-hold binding
    public Command clearThenStopCommand()
    {
        return startEnd(this::clear, this::stopIndexer);
    }

    public void clear()
    {
        setIndexerVelocity(
            SmartDashboard.getNumber(DashboardConstants.LAUNCHER_CLEAR_KEY, LauncherConstants.CLEAR_VELOCITY_RPM));
    }

    // Set shooter velocity in RPM
    private void setShooterVelocity(double velocity)
    {
        targetVelocity = velocity;
        leftShooterMotor.setControl(velocityVoltage.withVelocity(velocity / 60));
        atTargetVelocity = false;
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
        return runOnce(this::stopIndexer);
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

    public boolean isAtTargetVelocity()
    {
        return true;
        // TODO: determine if shooter is at target velocity within some tolerance?
    }

    // Stops all 3 motors in launcher
    public void stopAll()
    {
        stopIndexer();
        stopShooters();
    }

    public boolean isAtShooterVelocity()
    {
        return atTargetVelocity;
    }

    @Override
    public void periodic()
    {
        atTargetVelocity = Math.abs(
            leftShooterMotor.getVelocity().getValue().in(RPM) - targetVelocity) < LauncherConstants.SHOOTER_TOLERANCE_RPM;

        SmartDashboard.putNumber("ShooterV", leftShooterMotor.getVelocity().getValue().in(RPM));
        SmartDashboard.putNumber("IndexerV", indexerMotor.getVelocity().getValue().in(RPM));
    }
}
