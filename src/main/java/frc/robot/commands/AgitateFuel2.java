// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

/*
 * Implements a simple agitator that vibrates the intake back and forth
 * around its current position.
 */

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AgitateFuel2 extends Command
{
    // Create constants for agitate angle and movement duration.
    private final Intake intake;
    private double agitateAngle = 5; // angle to swing the intake
    private double startPosition = 0; // position of the intake at the time the agitation starts
    private long lastSwitchTime = 0; // used to track how long the agitator is moving in a direction
    private int movementDuration = 500;
    private boolean movementCommandIssued = false;

    private enum State
    {
        MOVING_UP, MOVING_DOWN
    }

    private State state;

    /** Creates a new AgitateFuel. */
    public AgitateFuel2(Intake intake)
    {
        this.intake = intake;
        addRequirements(intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {
        // grab the current position of the intake, we're going to agitate around
        // that position
        startPosition = intake.getPivotPosition().in(Units.Degrees);
        System.out.println("Agitate2::initialize, startPosition: " + startPosition);
        // make sure the range we vibrate the intake is in the range of
        // the intakes movement
        if (startPosition - agitateAngle < IntakeConstants.MIN_POSITION)
        {
            startPosition += agitateAngle;
        }
        else if (startPosition + agitateAngle > IntakeConstants.MAX_POSITION)
        {
            startPosition -= agitateAngle;
        }

        System.out.println("Agitate2::initialize, startPosition adjusted: " + startPosition);

        // grab the current time and set the initial state
        lastSwitchTime = System.currentTimeMillis();
        state = State.MOVING_DOWN;
        movementCommandIssued = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {
        // keep moving in a direction until the movement duration is exceeded
        // AND the intake is not stalled
        // once movement duration exceeded OR if he motor is stalled, switch direction

        // the effect should be that the intake quickly toggles back and forth
        // at its current position by the agitate angle for the movement duration

        long currentTime = System.currentTimeMillis();
        if (currentTime - lastSwitchTime > movementDuration && !intake.isPivotStalled())
        {
            // toggle the state every time the time expires and reset the time to
            // the current time
            state = state == State.MOVING_DOWN ? State.MOVING_UP : State.MOVING_DOWN;
            lastSwitchTime = currentTime;
            // toggle command issued so the other branch can issue the movement command.
            movementCommandIssued = false;
        }
        else
        {
            if (movementCommandIssued == false)
            {
                if (state == State.MOVING_UP)
                {
                    intake.pivotToPositionInDegrees(startPosition - agitateAngle);
                }
                else if (state == State.MOVING_DOWN)
                {
                    intake.pivotToPositionInDegrees(startPosition + agitateAngle);
                }
                movementCommandIssued = true;
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted)
    {
        intake.stop();
    }

    // Returns true when the command has ended.
    @Override
    public boolean isFinished()
    {
        // this command doesn't finish on its own, always report false
        return false;
    }
}
