// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import edu.wpi.first.wpilibj.Servo;

public class Climber extends SubsystemBase
{
    private final Servo servo = new Servo(ClimberConstants.SERVO_CHANNEL);
    private int currentServoPosition;

    /** Creates a new Climber. */
    public Climber()
    {
        currentServoPosition = ClimberConstants.RATCHET_ENGAGED_POSITION;
        servo.setPulseTimeMicroseconds(currentServoPosition);
    }

    public void setClimbSpeed(double speed)
    {
        // Set the winch motor to a specific speed (positive or negative)
    }

    public void toggleRatchet()
    {
        var position = (currentServoPosition == ClimberConstants.RATCHET_ENGAGED_POSITION)
            ? ClimberConstants.RATCHET_RELEASED_POSITION
            : ClimberConstants.RATCHET_ENGAGED_POSITION;
        servo.setPulseTimeMicroseconds(position);
        currentServoPosition = position;
    }

    public boolean isRatchetEngaged()
    {
        return currentServoPosition == ClimberConstants.RATCHET_ENGAGED_POSITION;
    }

    public double getPosition()
    {
        // Return the current position of the winch
        return currentServoPosition;
    }

    public void stop()
    {
        // Stop the winch motor
    }

    @Override
    public void periodic()
    {
        // This method will be called once per scheduler run
    }
}
