// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DashboardConstants;
import frc.robot.Constants.HopperConstants;

public class Hopper extends SubsystemBase
{
    private TalonFX indexerMotor = new TalonFX(Constants.HopperConstants.INDEXER_MOTOR_CHANNEL,
        Constants.HopperConstants.INDEXER_MOTOR_BUS);

    /** Creates a new Hopper. */
    public Hopper()
    {
        var configs = new TalonFXConfiguration();
        configs.MotorOutput.Inverted = Constants.HopperConstants.INDEXER_MOTOR_INVERTED_VALUE;
        indexerMotor.getConfigurator().apply(configs);
    }

    // Set the lower indexer motor to a specific speed
    private void setIndexerSpeed(double speed)
    {
        indexerMotor.set(speed);
    }

    // Feed fuel from the hopper to the launcher
    public void feed()
    {
        setIndexerSpeed(
            SmartDashboard.getNumber(DashboardConstants.HOPPER_INDEXER_FEED_KEY, HopperConstants.FEED_SPEED));
    }

    // Clear the hopper by running the feeding motor in reverse
    public void clear()
    {
        setIndexerSpeed(
            SmartDashboard.getNumber(DashboardConstants.HOPPER_INDEXER_CLEAR_KEY, HopperConstants.CLEAR_SPEED));
    }

    // Stop the indexer motor
    public void stop()
    {
        indexerMotor.stopMotor();
    }

    @Override
    public void periodic()
    {
        // This method will be called once per scheduler run
    }
}
