// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.Constants.AutomationConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreFuel extends SequentialCommandGroup
{
    /** Creates a new ScoreFuel. */
    public ScoreFuel(Launcher launcher, Hopper hopper, Intake intake)
    {
        addRequirements(launcher, hopper, intake);

        addCommands(
            new FunctionalCommand(() -> launcher.spinUpShooters(), () ->
            {
            }, (b) ->
            {
            }, () -> launcher.isAtTargetVelocity(), launcher),
            launcher.feedCommand(),
            hopper.feedCommand(),
            new WaitCommand(AutomationConstants.AGITATE_START_DELAY_SECS),
            new AgitateFuel(intake),
            launcher.stopShootersCommand(),
            launcher.stopIndexerCommand(),
            hopper.stopCommand());
    }
}
