// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase
{
    /** Creates a new Intake. */
    public Intake()
    {
    }

    public void setIntakeSpeed(double speed)
    {
        // Set the intake motor to a specific speed (positive or negative) in RPM
    }

    private void pivotToPosition(double position)
    {
        // Pivot the intake to a specific position
    }

    public void stow()
    {
        // Pivot the intake into "stowed" position
    }

    public void deploy()
    {
        // Pivot the intake into "deployed" position
    }

    public void partialDeploy()
    {
        // Pivot the intake into "partially deployed" position
    }

    public void intake()
    {
        // Ensure intake is pivoted into "deployed" position, then run the intake motor to pull fuel into the robot
    }

    public void eject()
    {
        // Ensure intake is pivoted into "deployed" position, then run the intake motor in reverse to "dump" fuel out of
        // the robot
    }

    public void stop()
    {
        // Stop the intake motor
    }

    @Override
    public void periodic()
    {
        // This method will be called once per scheduler run
    }
}
