// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutomationConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AgitateFuel extends Command
{

    private final Intake intake;
    private final Timer timer = new Timer();
    private int upPositionIndex = 0;

    private enum State
    {
        DOWN, MOVING_UP, UP, MOVING_DOWN
    }

    private State state;

    /** Creates a new AgitateFuel. */
    public AgitateFuel(Intake intake)
    {
        this.intake = intake;
        addRequirements(intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {
        upPositionIndex = 0;
        timer.restart();
        intake.deploy();
        state = State.MOVING_DOWN;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {
        SmartDashboard.putString("Agitate State", state.toString());
        switch (state)
        {
            case MOVING_DOWN:
                if (intake.atTargetPosition())
                {
                    state = State.DOWN;
                    timer.restart();
                }
                else if (intake.isPivotStalled())
                {
                    state = State.MOVING_UP;
                    intake.pivotToPositionInDegrees(AutomationConstants.AGITATE_UP_POSITIONS_DEG[upPositionIndex++]);
                    timer.restart();
                }
                break;

            case DOWN:
                if (timer.hasElapsed(AutomationConstants.AGITATE_DOWN_TIME_SECS))
                {
                    state = State.MOVING_UP;
                    intake.pivotToPositionInDegrees(AutomationConstants.AGITATE_UP_POSITIONS_DEG[upPositionIndex++]);
                }
                break;

            case MOVING_UP:
                if (intake.atTargetPosition())
                {
                    if (upPositionIndex > 0)
                    {
                        intake.setIntakeVelocity(IntakeConstants.AGITATE_VELOCITY_RPM);
                    }
                    state = State.UP;
                    timer.restart();
                }
                else if (intake.isPivotStalled())
                {
                    state = State.MOVING_DOWN;
                    intake.stop();
                    intake.deploy();
                }
                break;

            case UP:
                if (timer.hasElapsed(AutomationConstants.AGITATE_UP_TIME_SECS))
                {
                    state = State.MOVING_DOWN;
                    intake.stop();
                    intake.deploy();
                }
                break;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted)
    {
        intake.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return upPositionIndex >= AutomationConstants.AGITATE_UP_POSITIONS_DEG.length;
    }
}
