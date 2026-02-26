// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Climber;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveClimberWithGamepad extends Command
{
    private final Climber climber;
    private final CommandXboxController gamepad;

    /** Creates a new MoveClimberWithGamepad. */
    public MoveClimberWithGamepad(Climber climber, CommandXboxController gamnepad)
    {
        this.climber = climber;
        this.gamepad = gamnepad;

        addRequirements(climber);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {
        var speed = -gamepad.getLeftY() * ClimberConstants.WINCH_MAX_MAN_SPEED;
        var position = climber.getPosition();
        if ((speed < 0 && position <= ClimberConstants.MIN_POSITION) ||
            (speed > 0 && position >= ClimberConstants.MAX_POSITION))
        {
            speed = 0;
            gamepad.setRumble(RumbleType.kBothRumble, 1);
        }
        else
        {
            gamepad.setRumble(RumbleType.kBothRumble, 0);
        }
        climber.setWinchMotorSpeed(speed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted)
    {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return false;
    }
}
