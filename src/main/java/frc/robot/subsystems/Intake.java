// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.DashboardConstants;

public class Intake extends SubsystemBase
{

    private static TalonFX intakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR_CHANNEL);
    private static TalonFX pivotMotor = new TalonFX(IntakeConstants.PIVOT_MOTOR_CHANNEL);

    public static CANcoder pivotEncoder = new CANcoder(IntakeConstants.PIVOT_ENCODER_CHANNEL);

    private boolean isDeployed = false;

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
        pivotConfigs.Slot0.kP = IntakeConstants.IntakekP;
        pivotConfigs.Slot0.kI = IntakeConstants.IntakekI;
        pivotConfigs.Slot0.kD = IntakeConstants.IntakekD;
        pivotConfigs.MotorOutput.PeakForwardDutyCycle = IntakeConstants.PIVOT_SPEED;
        pivotConfigs.MotorOutput.PeakReverseDutyCycle = -IntakeConstants.PIVOT_SPEED;
        pivotConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pivotConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = IntakeConstants.DEPLOYED_POSITION;
        pivotConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        pivotConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = IntakeConstants.STOW_POSITION;
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

    // Pivot the intake to a specific position
    private void pivotToPosition(double position)
    {
        pivotMotor.setControl(new PositionDutyCycle(position).withSlot(0));
    }

    // Pivot the intake into "stowed" position
    public void stow()
    {
        pivotToPosition(IntakeConstants.STOW_POSITION);
    }

    // Pivot the intake into "deployed" position
    public void deploy()
    {
        pivotToPosition(IntakeConstants.DEPLOYED_POSITION);
    }

    // Pivot the intake into "partially deployed" position
    public void partialDeploy()
    {
        pivotToPosition(IntakeConstants.PARTIALLY_DEPLOYED_POSITION);
    }

    // Ensure intake is pivoted into "deployed" position, then run the intake motor to pull fuel into the robot
    public void intake()
    {
            setIntakeSpeed(
                SmartDashboard.getNumber(DashboardConstants.INTAKE_KEY, IntakeConstants.INTAKE_SPEED));
    }

    // Ensure intake is pivoted into "deployed" position, then run the intake motor in reverse to "dump" fuel out of
    // the robot
    public void eject()
    {
            setIntakeSpeed(
                SmartDashboard.getNumber(DashboardConstants.EJECT_KEY, IntakeConstants.EJECT_SPEED));
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
        double currentPivotPosition = pivotEncoder.getPosition().getValueAsDouble();

        if (currentPivotPosition > IntakeConstants.DEPLOYED_POSITION - 10)
        {
            isDeployed = true;
        }
        else
        {
            isDeployed = false;
        }
    }
}
