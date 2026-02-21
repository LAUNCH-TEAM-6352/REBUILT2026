// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.TestConstants;
import frc.robot.subsystems.Intake;

/**
 * Test Intake functionaliuty.
 */
public class TestIntake extends SequentialCommandGroup
{
    /** Creates a new TestIntake. */
    public TestIntake(Intake intake)
    {
        addRequirements(intake);
        addCommands(
            new InstantCommand(() -> System.out.println("Testing Intake: Patrially Deploying")),
            intake.partialDeployCommand(),
            new WaitCommand(TestConstants.BETWEEN_TIME_SECS),

            new InstantCommand(() -> System.out.println("Testing Intake: Fully Deploying")),
            intake.deployCommand(),
            new WaitCommand(TestConstants.BETWEEN_TIME_SECS),

            new InstantCommand(() -> System.out.println("Testing Intake: Intaking")),
            intake.intakeThenStopCommand().withTimeout(TestConstants.INSTANT_BETWEEN_TIME_SECS),
            new WaitCommand(TestConstants.BETWEEN_TIME_SECS),

            new InstantCommand(() -> System.out.println("Testing Intake: Ejecting")),
            intake.ejectThenStopCommand().withTimeout(TestConstants.INSTANT_BETWEEN_TIME_SECS),
            new WaitCommand(TestConstants.BETWEEN_TIME_SECS),

            new InstantCommand(() -> System.out.println("Testing Intake: Stowing")),
            intake.stowCommand(),
            new WaitCommand(TestConstants.BETWEEN_TIME_SECS));
    }
}
