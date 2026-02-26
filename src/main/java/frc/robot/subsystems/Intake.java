// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DashboardConstants;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase
{
    private final TalonFX intakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR_CHANNEL,
        IntakeConstants.INTAKE_MOTOR_BUS);
    private final TalonFX pivotMotor = new TalonFX(IntakeConstants.PIVOT_MOTOR_CHANNEL,
        IntakeConstants.PIVOT_MOTOR_BUS);

    private final CANcoder pivotEncoder = new CANcoder(IntakeConstants.PIVOT_ENCODER_CHANNEL,
        IntakeConstants.PIVOT_ENCODER_BUS);

    private final PositionDutyCycle positionControl = new PositionDutyCycle(0);

    /** Creates a new Intake. */
    public Intake()
    {
        var intakeConfigs = new TalonFXConfiguration();
        intakeConfigs.MotorOutput.Inverted = IntakeConstants.INTAKE_MOTOR_INVERTED_VALUE;
        intakeMotor.getConfigurator().apply(intakeConfigs);

        var pivotConfigs = new TalonFXConfiguration();
        pivotConfigs.MotorOutput.Inverted = IntakeConstants.PIVOT_MOTOR_INVERTED_VALUE;

        pivotConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        pivotConfigs.Feedback.FeedbackRemoteSensorID = pivotEncoder.getDeviceID();

        pivotConfigs.Slot0.kP = IntakeConstants.INTAKE_KP;
        pivotConfigs.Slot0.kI = IntakeConstants.INTAKE_KI;
        pivotConfigs.Slot0.kD = IntakeConstants.INTAKE_KD;

        pivotConfigs.MotorOutput.PeakForwardDutyCycle = IntakeConstants.PIVOT_MAX_FWD_SPEED;
        pivotConfigs.MotorOutput.PeakReverseDutyCycle = IntakeConstants.PIVOT_MAX_REV_SPEED;
        pivotConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        pivotConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = IntakeConstants.MAX_POSITION;
        pivotConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        pivotConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = IntakeConstants.MIN_POSITION;
        pivotConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        pivotMotor.getConfigurator().apply(pivotConfigs);

        var canCoderConfig = new CANcoderConfiguration();
        canCoderConfig.MagnetSensor.SensorDirection = IntakeConstants.ENCODER_DIRECTION_VALUE;
        pivotEncoder.getConfigurator().apply(canCoderConfig);

        pivotMotor.clearStickyFaults();
        pivotEncoder.setPosition(0);
    }

    // Set the intake motor to a specific speed
    private void setIntakeSpeed(double speed)
    {
        intakeMotor.set(speed);
    }

    // Pivots the intake to a specified position, specified in degrtees.
    private void pivotToPositionInDegrees(double position)
    {
        pivotMotor.setControl(positionControl.withPosition(Degrees.of(position)));
    }

    public Command stowCommand()
    {
        return runOnce(this::stow);
    }

    public void stow()
    {
        pivotToPositionInDegrees(
            SmartDashboard.getNumber(DashboardConstants.STOWED_KEY, IntakeConstants.STOWED_POSITION.magnitude()));
    }

    public Command deployCommand()
    {
        return runOnce(this::deploy);
    }

    public void deploy()
    {
        pivotToPositionInDegrees(
            SmartDashboard.getNumber(DashboardConstants.DEPLOYED_KEY, IntakeConstants.DEPLOYED_POSITION.magnitude()));
    }

    public Command partialDeployCommand()
    {
        return runOnce(this::partialDeploy);
    }

    public void partialDeploy()
    {
        pivotToPositionInDegrees(SmartDashboard.getNumber(DashboardConstants.PARTIALLY_DEPLOYED_KEY,
            IntakeConstants.PARTIALLY_DEPLOYED_POSITION.magnitude()));
    }

    // Intended for use with a press-and-hold binding
    public Command intakeThenStopCommand()
    {
        return startEnd(this::intake, this::stop);
    }

    // Intended for use in auto (Only starts the intake, does not stop automatically)
    public Command intakeCommand()
    {
        return runOnce(this::intake);
    }

    public void intake()
    {
        setIntakeSpeed(
            SmartDashboard.getNumber(DashboardConstants.INTAKE_SPEED_KEY, IntakeConstants.INTAKE_SPEED));
    }

    // Intended for use with a press-and-hold binding
    public Command ejectThenStopCommand()
    {
        return startEnd(this::eject, this::stop);
    }

    public void eject()
    {
        // TODO: Determine if intake is at desired position within some tolerance?
        setIntakeSpeed(
            SmartDashboard.getNumber(DashboardConstants.EJECT_SPEED_KEY, IntakeConstants.EJECT_SPEED));
    }

    // Stop the intake motor
    public void stop()
    {
        intakeMotor.stopMotor();
    }

    @Override
    public void periodic()
    {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Intake Pos", pivotEncoder.getAbsolutePosition().getValue().in(Degrees));
    }
}
