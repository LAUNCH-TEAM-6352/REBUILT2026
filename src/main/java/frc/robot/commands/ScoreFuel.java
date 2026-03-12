// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Hopper;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreFuel extends SequentialCommandGroup {
    /** Creates a new ScoreFuel. */
    public ScoreFuel(Launcher launcher, Hopper hopper) {
        addRequirements(launcher, hopper);

        addCommands(
                new FunctionalCommand(() -> launcher.spinUpShooters(), () -> {
                }, (b) -> {
                }, () -> launcher.isShooterAtVelocity(), launcher),

                new FunctionalCommand(() -> launcher.feed(), () -> {
                }, (b) -> {
                }, () -> launcher.isIndexerAtVelocity(), launcher),

                launcher.feedCommand());
    }
}
