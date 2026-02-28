// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DashboardConstants;
import frc.robot.Constants.HopperConstants;

public class Hopper extends SubsystemBase
{
    private final TalonFX conveyorMotor = new TalonFX(Constants.HopperConstants.CONVEYOR_MOTOR_CHANNEL,
        Constants.HopperConstants.CONVEYOR_MOTOR_BUS);

    /** Creates a new Hopper. */
    public Hopper()
    {
        var configs = new TalonFXConfiguration();
        configs.MotorOutput.Inverted = Constants.HopperConstants.CONVEYOR_MOTOR_INVERTED_VALUE;
        conveyorMotor.getConfigurator().apply(configs);
        conveyorMotor.clearStickyFaults();
    }

    // Set the lower indexer motor to a specific speed
    private void setConveyor(double speed)
    {
        conveyorMotor.set(speed);
    }

    // Intended for use with a press-and-hold binding
    public Command feedThenStopCommand()
    {
        return startEnd(this::feed, this::stop);
    }

    // Intended for use in auto (Only starts feeding, does not stop automatically)
    public Command feedCommand()
    {
        return runOnce(this::feed);
    }

    // Feed fuel from the hopper to the launcher
    public void feed()
    {
        setConveyor(
            SmartDashboard.getNumber(DashboardConstants.CONVEYOR_FEED_KEY, HopperConstants.FEED_SPEED));
    }

    // Intended for use with a press-and-hold binding
    public Command clearThenStopCommand()
    {
        return startEnd(this::clear, this::stop);
    }

    // Clear the hopper by running the feeding motor in reverse
    public void clear()
    {
        setConveyor(
            SmartDashboard.getNumber(DashboardConstants.CONVEYOR_CLEAR_KEY, HopperConstants.CLEAR_SPEED));
    }

    // Stop the indexer motor
    public void stop()
    {
        conveyorMotor.stopMotor();
    }

    @Override
    public void periodic()
    {
        // This method will be called once per scheduler run
    }
}
