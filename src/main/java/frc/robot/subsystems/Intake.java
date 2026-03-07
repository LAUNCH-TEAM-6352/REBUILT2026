// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
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

    private final VelocityVoltage velocityControl = new VelocityVoltage(0).withSlot(0);
    private final PositionVoltage positionControl = new PositionVoltage(0).withSlot(0);

    private double targetPosition;
    private double targetTolerance;
    private boolean atTargetPosition = false;
    private boolean isPositioningStarted;

    /** Creates a new Intake. */
    public Intake()
    {
        var intakeConfigs = new TalonFXConfiguration();
        intakeConfigs.MotorOutput.Inverted = IntakeConstants.INTAKE_MOTOR_INVERTED_VALUE;
        var slot0Configs = new Slot0Configs();
        slot0Configs.kS = IntakeConstants.INTAKE_KS;
        slot0Configs.kV = IntakeConstants.INTAKE_KV;
        slot0Configs.kP = IntakeConstants.INTAKE_KP;
        slot0Configs.kI = IntakeConstants.INTAKE_KI;
        slot0Configs.kD = IntakeConstants.INTAKE_KD;
        intakeMotor.getConfigurator().apply(intakeConfigs);
        intakeMotor.getConfigurator().apply(slot0Configs);
        intakeMotor.clearStickyFaults();

        var pivotConfigs = new TalonFXConfiguration();
        pivotConfigs.MotorOutput.Inverted = IntakeConstants.PIVOT_MOTOR_INVERTED_VALUE;

        pivotConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        pivotConfigs.Feedback.FeedbackRemoteSensorID = pivotEncoder.getDeviceID();

        pivotConfigs.Slot0.kP = IntakeConstants.PIVOT_KP;
        pivotConfigs.Slot0.kI = IntakeConstants.PIVOT_KI;
        pivotConfigs.Slot0.kD = IntakeConstants.PIVOT_KD;
        pivotConfigs.Slot0.kG = IntakeConstants.PIVOT_KG;
        pivotConfigs.Slot0.kS = IntakeConstants.PIVOT_KS;
        pivotConfigs.Slot0.StaticFeedforwardSign = IntakeConstants.PIVOT_STATIC_FF_SIGN;
        pivotConfigs.Slot0.GravityType = IntakeConstants.PIVOT_GRAVITY_TYPE;
        pivotConfigs.Slot0.GravityArmPositionOffset = IntakeConstants.PIVOT_GRAVITY_ARM_POSITION_OFFSET;

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
        pivotEncoder.setPosition(IntakeConstants.STOWED_POSITION);
    }

    // Set the intake motor to a specific velocity in RPM
    public void setIntakeVelocity(double velocity)
    {
        intakeMotor.setControl(velocityControl.withVelocity(velocity / 60));
    }

    // Set the piviot motor to a specific speed (intended for manual control, not position control)
    public void setPivotMotorSpeed(double speed)
    {
        pivotMotor.set(speed);
    }

    public void setPivotMotorVolts(double volts)
    {
        pivotMotor.setVoltage(volts);
    }

    // Pivots the intake to a specified position, specified in degrtees.
    public void pivotToPositionInDegrees(double position)
    {
        pivotMotor.setControl(positionControl.withPosition(Degrees.of(position)));
        targetPosition = position;
        targetTolerance = IntakeConstants.PIVOT_TOLERANCE_DEG;
        isPositioningStarted = true;
        atTargetPosition = false;
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
        setIntakeVelocity(
            SmartDashboard.getNumber(DashboardConstants.INTAKE_VELOCITY_KEY, IntakeConstants.INTAKE_VELOCITY_RPM));
    }

    // Stop the intake motor
    public void stop()
    {
        intakeMotor.stopMotor();
    }

    // Returns the current pivot position as an angle relative to the stowed position.
    public Angle getPivotPosition()
    {
        return pivotEncoder.getAbsolutePosition().getValue();
    }

    public boolean atTargetPosition()
    {
        return atTargetPosition;
    }

    @Override
    public void periodic()
    {
        var pivotPosition = getPivotPosition().in(Degrees);

        if (isPositioningStarted)
        {
            if (Math.abs(pivotPosition - targetPosition) <= targetTolerance)
            {
                atTargetPosition = true;
                isPositioningStarted = false;
            }
        }

        SmartDashboard.putNumber("Intake Pos", getPivotPosition().in(Degrees));
        SmartDashboard.putNumber("IntakeOut", intakeMotor.getDutyCycle().getValueAsDouble());
        SmartDashboard.putNumber("IntakeRPM", intakeMotor.getVelocity().getValue().in(RPM));
        SmartDashboard.putNumber("PivotSpd", pivotMotor.getDutyCycle().getValueAsDouble());
        SmartDashboard.putNumber("PivotV", pivotMotor.getMotorVoltage().getValueAsDouble());
    }
}
