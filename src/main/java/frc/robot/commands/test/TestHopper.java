// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.TestConstants;
import frc.robot.subsystems.Hopper;

/**
 * Test Hopper functionaliuty.
 */
public class TestHopper extends SequentialCommandGroup
{
    /** Creates a new TestHopper. */
    public TestHopper(Hopper hopper)
    {
        addRequirements(hopper);
        addCommands(
            new InstantCommand(() -> System.out.println("Testing Hopper: Feeding")),
            hopper.feedThenStopCommand().withTimeout(TestConstants.INSTANT_BETWEEN_TIME_SECS),
            new WaitCommand(TestConstants.BETWEEN_TIME_SECS),

            new InstantCommand(() -> System.out.println("Testing Hopper: Clearing")),
            hopper.clearThenStopCommand().withTimeout(TestConstants.INSTANT_BETWEEN_TIME_SECS),
            new WaitCommand(TestConstants.BETWEEN_TIME_SECS));
    }
}
