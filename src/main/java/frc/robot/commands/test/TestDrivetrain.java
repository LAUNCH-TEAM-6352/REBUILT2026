// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.TestConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class TestDrivetrain extends SequentialCommandGroup
{
    /** Creates a new TestDriveTrain. */
    public TestDrivetrain(CommandSwerveDrivetrain drivetrain)
    {
        for (int i = 0; i < 4; i++)
        {
            addCommands(
                new TestSwerveDriveModule(drivetrain, i),
                new WaitCommand(TestConstants.BETWEEN_TIME_SECS));
        }
    }
}
