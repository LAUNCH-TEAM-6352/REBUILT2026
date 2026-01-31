// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
    // Subsystems:
    private final Optional<Climber> climber;
    private final Optional<Launcher> launcher;
    private final Optional<Intake> intake;
    private final Optional<Hopper> hopper;

    // OI devices:
    private XboxController driverGamepad;
    private final XboxController codriverGamepad;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer()
    {
        // Get the game data message fom the driver station.
        // This message is primarily used during development to
        // construct only certain subsystems.
        // If the message is blank (or all whitespace),
        // all subsystems are constructed.
        // Otherwise, OI devices and subsystems are constructed
        // depending upon the substrings found in the message:
        // -dt- Drive train
        // -oi- OI devices
        // -cl- Climber
        // -l- Launcher
        // -i- Intake
        // -h- Hopper

        var gameData = DriverStation.getGameSpecificMessage().toLowerCase();
        SmartDashboard.putString("Game Data", gameData);

        // Create OI devices:
        if (gameData.contains("-oi-"))
        {
            // Explicitly look for OI devices:
            driverGamepad = DriverStation.isJoystickConnected(OperatorConstants.DRIVER_GAMEPAD_PORT)
                ? new XboxController(OperatorConstants.DRIVER_GAMEPAD_PORT)
                : null;
            codriverGamepad = DriverStation.isJoystickConnected(OperatorConstants.CODRIVER_GAMEPAD_PORT)
                ? new XboxController(OperatorConstants.CODRIVER_GAMEPAD_PORT)
                : null;
        }
        else
        {
            // In competition, don't take chances and always create all OI devices:
            codriverGamepad = new XboxController(OperatorConstants.CODRIVER_GAMEPAD_PORT);
            driverGamepad = new XboxController(OperatorConstants.DRIVER_GAMEPAD_PORT);
        }

        // Create subsystems:
        climber = (gameData.contains("-cl-") || gameData.isBlank())
            ? Optional.of(new Climber())
            : Optional.empty();
        launcher = (gameData.contains("-l-") || gameData.isBlank())
            ? Optional.of(new Launcher())
            : Optional.empty();
        intake = (gameData.contains("-i-") || gameData.isBlank())
            ? Optional.of(new Intake())
            : Optional.empty();
        hopper = (gameData.contains("-h-") || gameData.isBlank())
            ? Optional.of(new Hopper())
            : Optional.empty();
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings()
    {

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        // An example command will be run in autonomous
        return Autos.exampleAuto();
    }
}
