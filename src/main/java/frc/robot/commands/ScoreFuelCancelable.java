package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Launcher;

public class ScoreFuelCancelable extends Command
{

    private Launcher launcher;
    private Hopper hopper;

    public ScoreFuelCancelable(Launcher launcher, Hopper hopper)
    {
        this.hopper = hopper;
        this.launcher = launcher;
    }

    @Override
    public void initialize()
    {
        // initial state of the launcher and hopper is stopped
        launcher.stopAll();
        hopper.stop();

        super.initialize();
    }

    @Override
    public void end(boolean interrupted)
    {
        // stop everything regarless of whether we were interrupted or not
        launcher.stopAll();
        hopper.stop();

        super.end(interrupted);
    }

    @Override
    public void execute()
    {
        launcher.spinUpShooters();
        launcher.feed();
        hopper.feed();
        super.execute();
    }
}
