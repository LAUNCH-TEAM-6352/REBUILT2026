package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Launcher;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ScoreFuelCancelableParallelGroup extends Command
{

    private Launcher launcher;
    private Hopper hopper;
    Command command = Commands.sequence(
        launcher.spinUpShootersCommand(),
        new WaitCommand(3.0),
        Commands.parallel(launcher.feedCommand(), hopper.feedCommand()));

    public ScoreFuelCancelableParallelGroup(Launcher launcher, Hopper hopper)
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
        // launcher.stopAll();
        // hopper.stop();
        command.cancel();
        super.end(interrupted);
    }

    @Override
    public void execute()
    {
        command.schedule();

        super.execute();
    }
}
