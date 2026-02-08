// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Launcher extends SubsystemBase
{

    private final TalonFX shooterLeft = new TalonFX(60);
    private final TalonFX shooterRight = new TalonFX(70);

    /** Creates a new Launcher. */
    public Launcher()
    {
        // start with factory-default configs
        var currentConfigs = new MotorOutputConfigs();

        // The right motor is CW+
        currentConfigs.Inverted = InvertedValue.Clockwise_Positive;
        shooterRight.getConfigurator().apply(currentConfigs);

        // Ensure our followers are following their respective leader
        shooterRight.setControl(new Follower(shooterLeft.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    public Command setLaunchSpeed(double speed)
    {
        // Set Launch Speed
        return this.runOnce(() ->
        {
            shooterRight.set(speed);
        });
    }

    @Override
    public void periodic()
    {
        // This method will be called once per scheduler run
    }
}
