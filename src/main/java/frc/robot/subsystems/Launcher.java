// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Launcher extends SubsystemBase
{
    /** Creates a new Launcher. */
    public Launcher()
    {
    }

    public void Feed()
    {
        // Confirm shooter motors are at speed, then
        // Run the upper indexer motor to move fuel to the shooter
    }

    public void Clear()
    {
        // Clear the launcher by running the upper indexer motor in reverse
    }

    public void SpinUpShooters(double speed)
    {
        // Set left and right shooter motors to a specific speed in RPM
    }

    public void IdleFeeders()
    {
        // Set left and right shooter motors to a low "idle" speed to keep shooter spinning
    }

    public void StopIndexer()
    {
        // Stop upper indexer motor
    }

    public void StopShooters()
    {
        // Stop left and right shooter motors
    }

    // Stops all 3 motors in launcher
    public void Stop()
    {
        // Stop the upper indexer motor
        StopIndexer();
        // Stop lefts and right shooter motors
        StopShooters();
    }

    @Override
    public void periodic()
    {
        // This method will be called once per scheduler run
    }
}
