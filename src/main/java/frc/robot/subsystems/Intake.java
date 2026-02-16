// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DashboardConstants;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase
{

    private final TalonFX intakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR_CHANNEL);
    private final TalonFX pivotMotor = new TalonFX(IntakeConstants.PIVOT_MOTOR_CHANNEL);

    private final CANcoder pivotEncoder = new CANcoder(IntakeConstants.PIVOT_ENCODER_CHANNEL);

    private final PositionVoltage positionVoltage = new PositionVoltage(0);

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
        pivotConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pivotConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 1
            * (SmartDashboard.getNumber(DashboardConstants.DEPLOY_KEY, IntakeConstants.DEPLOYED_POSITION) / 360.0);
        pivotConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        pivotConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 1
            * (SmartDashboard.getNumber(DashboardConstants.STOW_KEY, IntakeConstants.STOW_POSITION) / 360.0);
        pivotConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        pivotMotor.getConfigurator().apply(pivotConfigs);

        var canCoderConfig = new CANcoderConfiguration();
        canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        pivotEncoder.getConfigurator().apply(canCoderConfig);

        pivotMotor.clearStickyFaults();
        pivotEncoder.setPosition(0);
    }

    // Set the intake motor to a specific speed
    private void setIntakeSpeed(double speed)
    {
        intakeMotor.set(speed);
    }

    private void pivotToPosition(double position)
    {
        pivotMotor.setControl(positionVoltage.withPosition(position));
    }

    public Command stowCommand()
    {
        return runOnce(this::stow);
    }

    public void stow()
    {
        pivotToPosition(SmartDashboard.getNumber(DashboardConstants.STOW_KEY, IntakeConstants.STOW_POSITION));
    }

    public Command deployCommand()
    {
        return runOnce(this::deploy);
    }

    public void deploy()
    {
        pivotToPosition(SmartDashboard.getNumber(DashboardConstants.DEPLOY_KEY, IntakeConstants.DEPLOYED_POSITION));
    }

    public Command partialDeployCommand()
    {
        return runOnce(this::partialDeploy);
    }

    public void partialDeploy()
    {
        pivotToPosition(SmartDashboard.getNumber(DashboardConstants.PARTIALLY_DEPLOY_KEY,
            IntakeConstants.PARTIALLY_DEPLOYED_POSITION));
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
    }
}
