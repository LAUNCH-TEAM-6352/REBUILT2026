// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hopper extends SubsystemBase
{
    /** Creates a new Hopper. */
    public Hopper()
    {
    }

    private void setIndexerSpeed(double speed)
    {
        // Set the lower indexer motor to a specific speed in RPM
    }

    public void feed()
    {
        // Run the feeding motor to move fuel from the intake to the shooter
    }

    public void clear()
    {
        // Clear the hopper by running the feeding motor in reverse
    }

    public void stop()
    {
        // Stop the feeding motor
    }

    @Override
    public void periodic()
    {
        // This method will be called once per scheduler run
    }
}
