// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DashboardConstants;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climber extends SubsystemBase
{
    private final TalonFX winchMotor = new TalonFX(ClimberConstants.WINCH_MOTOR_CHANNEL,
        ClimberConstants.WINCH_MOTOR_BUS);

    private final Servo servo = new Servo(ClimberConstants.SERVO_CHANNEL);

    private final DutyCycleOut dutyCycleControl = new DutyCycleOut(0);
    private final PositionDutyCycle positionControl = new PositionDutyCycle(0);

    /** Creates a new Climber. */
    public Climber()
    {
        var configs = new TalonFXConfiguration();
        configs.Slot0.kP = ClimberConstants.WINCH_KP;
        configs.Slot0.kI = ClimberConstants.WINCH_KI;
        configs.Slot0.kD = ClimberConstants.WINCH_KD;

        configs.MotorOutput.Inverted = ClimberConstants.WINCH_MOTOR_INVERTED_VALUE;

        configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ClimberConstants.CLIMBED_POSITION;
        configs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ClimberConstants.STOWED_POSITION;
        configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        configs.MotorOutput.PeakForwardDutyCycle = ClimberConstants.WINCH_MAX_FWD_SPEED;
        configs.MotorOutput.PeakReverseDutyCycle = ClimberConstants.WINCH_MAX_REV_SPEED;

        winchMotor.getConfigurator().apply(configs);
        winchMotor.setPosition(0);

        servo.setPosition(ClimberConstants.RATCHET_ENGAGED_POSITION);
    }

    public void setWinchMotorSpeed(double speed)
    {
        winchMotor.setControl(dutyCycleControl.withOutput(speed));
    }

    private void setPosition(double position)
    {
        winchMotor.setControl(positionControl.withPosition(position));
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

    private void engageRatchet()
    {
        servo.setPosition(ClimberConstants.RATCHET_ENGAGED_POSITION);
    }

    private void releaseRatchet()
    {
        servo.setPosition(ClimberConstants.RATCHET_RELEASED_POSITION);
    }

    public Command engageRatchetCommand()
    {
        return runOnce(this::engageRatchet);
    }

    public Command releaseRatchetCommand()
    {
        return runOnce(this::releaseRatchet);
    }

    public double getPosition()
    {
        return winchMotor.getPosition().getValueAsDouble();
    }

    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("Climber Pos", winchMotor.getPosition().getValueAsDouble());
    }
}
