// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.LimeLightVision;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
    // The robot's subsystems and commands are defined here...
    private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

    // wrapper class to manage limelight cameras and get position estimates
    public final LimeLightVision limelightVision = new LimeLightVision(List.of("limelight-front"));

    // drive train
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController m_driverController = new CommandXboxController(
        OperatorConstants.kDriverControllerPort);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer()
    {
        // Configure the trigger bindings
        configureBindings();
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
        // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
        new Trigger(m_exampleSubsystem::exampleCondition)
            .onTrue(new ExampleCommand(m_exampleSubsystem));

        // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
        // cancelling on release.
        m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        // An example command will be run in autonomous
        return Autos.exampleAuto(m_exampleSubsystem);
    }

    public Command pathfindToPose(Pose2d point, Double endVelocity)
    {
        // Creates a command to pathfind to the given pose

        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
            5.0, 4.0,
            Units.degreesToRadians(540), Units.degreesToRadians(-180));

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        Command pathfindingCommand = AutoBuilder.pathfindToPose(
            point,
            constraints,
            endVelocity // Goal end velocity in meters/sec
        );
        return pathfindingCommand;
    }

    public void pathFindToMultiPose(List<Pose2d> points)
    {

        // var cmd = pathfindToPose(points.get(0), 10.0);
        var pointsIterator = points.iterator();
        Command cmd = pathfindToPose(pointsIterator.next(), 5.0);
        Command lastCommand = cmd;
        while (pointsIterator.hasNext())
        {
            lastCommand = lastCommand.andThen(pathfindToPose(pointsIterator.next(), 10.0));
        }

        lastCommand.schedule();
    }

    public void updateVisionEstimate()
    {
        // updatePoseEstimation function assigns estimations to the
        // drive train.
        this.limelightVision.updatePoseEstimation(drivetrain);
    }
}
