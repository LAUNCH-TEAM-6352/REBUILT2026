// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DashboardConstants;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climber extends SubsystemBase
{
    private final TalonFX winchMotor = new TalonFX(ClimberConstants.WINCH_MOTOR_CHANNEL);
    private final Servo servo = new Servo(ClimberConstants.SERVO_CHANNEL);

    private final PositionVoltage positionVoltage = new PositionVoltage(0);

    private int currentServoPosition;

    /** Creates a new Climber. */
    public Climber()
    {
        var configs = new TalonFXConfiguration();
        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = ClimberConstants.WINCH_KP;
        slot0Configs.kI = ClimberConstants.WINCH_KI;
        slot0Configs.kD = ClimberConstants.WINCH_KD;
        configs.MotorOutput.Inverted = ClimberConstants.WINCH_MOTOR_INVERTED_VALUE;
        winchMotor.getConfigurator().apply(configs);
        winchMotor.getConfigurator().apply(slot0Configs);

        currentServoPosition = ClimberConstants.RATCHET_ENGAGED_POSITION;
        servo.setPulseTimeMicroseconds(currentServoPosition);
    }

    private void setPosition(double position)
    {
        winchMotor.setControl(positionVoltage.withPosition(position));
    }

    public Command climbCommand()
    {
        return runOnce(this::climb);
    }

    public Command extendCommand()
    {
        return runOnce(this::extend);
    }

    public Command stowCommand()
    {
        return runOnce(this::stow);
    }

    public void climb()
    {
        setPosition(SmartDashboard.getNumber(DashboardConstants.CLIMBER_CLIMB_KEY, ClimberConstants.CLIMBED_POSITION));
    }

    public void extend()
    {
        setPosition(
            SmartDashboard.getNumber(DashboardConstants.CLIMBER_EXTEND_KEY, ClimberConstants.EXTENDED_POSITION));
    }

    public void stow()
    {
        setPosition(SmartDashboard.getNumber(DashboardConstants.CLIMBER_STOW_KEY, ClimberConstants.STOWED_POSITION));
    }

    public void toggleRatchet()
    {
        var position = isRatchetEngaged()
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
        return winchMotor.getPosition().getValueAsDouble();
    }

    @Override
    public void periodic()
    {
        // This method will be called once per scheduler run
    }
}
