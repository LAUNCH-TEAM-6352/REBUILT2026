// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.TestConstants;
import frc.robot.subsystems.Climber;

/**
 * Test Climber functionaliuty.
 */
public class TestClimber extends SequentialCommandGroup
{
    /** Creates a new TestClimber. */
    public TestClimber(Climber climber)
    {
        addRequirements(climber);
        addCommands(
            new InstantCommand(() -> System.out.println("Testing Climber: Extending")),
            climber.extendCommand(),
            new WaitCommand(TestConstants.BETWEEN_TIME_SECS),

            new InstantCommand(() -> System.out.println("Testing Climber: Climbing")),
            climber.climbCommand(),
            new WaitCommand(TestConstants.BETWEEN_TIME_SECS),

            new InstantCommand(() -> System.out.println("Testing Climber: Stowing")),
            climber.stowCommand(),
            new WaitCommand(TestConstants.BETWEEN_TIME_SECS),

            new InstantCommand(() -> System.out.println("Testing Climber Ratchet: Releasing")),
            new InstantCommand(() -> climber.toggleRatchet()),
            new WaitCommand(TestConstants.INSTANT_BETWEEN_TIME_SECS),

            new InstantCommand(() -> System.out.println("Testing Climber Ratchet: Engaging")),
            new InstantCommand(() -> climber.toggleRatchet()),
            new WaitCommand(TestConstants.INSTANT_BETWEEN_TIME_SECS),

            new WaitCommand(TestConstants.BETWEEN_TIME_SECS));
    }
}
