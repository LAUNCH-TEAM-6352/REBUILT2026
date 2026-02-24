// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.List;
import java.util.Optional;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DashboardConstants;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.test.TestClimber;
import frc.robot.Constants.PathPlannerConstants;
import frc.robot.commands.test.TestDrivetrain;
import frc.robot.commands.test.TestHopper;
import frc.robot.commands.test.TestIntake;
import frc.robot.commands.test.TestLauncher;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.LimeLightVision;
import frc.robot.subsystems.Hopper;
import com.pathplanner.lib.path.PathPlannerPath;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

public class RobotContainer
{
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                        // speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    // wrapper class to manage limelight cameras and get position estimates
    public final LimeLightVision limelightVision = new LimeLightVision(List.of("limelight-front"));

    // Subsystems:
    private final Optional<Climber> climber;
    private final Optional<Launcher> launcher;
    private final Optional<Intake> intake;
    private final Optional<Hopper> hopper;
    private final Optional<CommandSwerveDrivetrain> drivetrain;

    // OI devices:
    private final CommandXboxController driverGamepad;
    private final CommandXboxController codriverGamepad;

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
        // -c- Climber
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
                ? new CommandXboxController(OperatorConstants.DRIVER_GAMEPAD_PORT)
                : null;
            codriverGamepad = DriverStation.isJoystickConnected(OperatorConstants.CODRIVER_GAMEPAD_PORT)
                ? new CommandXboxController(OperatorConstants.CODRIVER_GAMEPAD_PORT)
                : null;
        }
        else
        {
            // In competition, don't take chances and always create all OI devices:
            codriverGamepad = new CommandXboxController(OperatorConstants.CODRIVER_GAMEPAD_PORT);
            driverGamepad = new CommandXboxController(OperatorConstants.DRIVER_GAMEPAD_PORT);
        }
        // Create subsystems:
        climber = (gameData.contains("-c-") || gameData.isBlank() || gameData.length() == 1)
            ? Optional.of(new Climber())
            : Optional.empty();
        launcher = (gameData.contains("-l-") || gameData.isBlank() || gameData.length() == 1)
            ? Optional.of(new Launcher())
            : Optional.empty();
        intake = (gameData.contains("-i-") || gameData.isBlank() || gameData.length() == 1)
            ? Optional.of(new Intake())
            : Optional.empty();
        hopper = (gameData.contains("-h-") || gameData.isBlank() || gameData.length() == 1)
            ? Optional.of(new Hopper())
            : Optional.empty();

        drivetrain = (gameData.contains("-dt-") || gameData.isBlank() || gameData.length() == 1)
            ? Optional.of(TunerConstants.createDrivetrain())
            : Optional.empty();

        if (drivetrain.isPresent())
        {
            drivetrain.get().setupPathPlanner();
        }

        configurePathPlannerNamedCommands();

        configureBindings();

        // Configure dashboard values
        configureDashboard();
    }

    private void configurePathPlannerNamedCommands()
    {
        // Register any commands that should be available in PathPlanner's command builder here.
        // For example:
        // NamedCommands.registerCommand("runShoot", new RunLauncher(launcher));

        // TODO: replace command with command for running the launcher
        NamedCommands.registerCommand("runShoot", Commands.runOnce(() -> System.out.println("booger2")));
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
        climber.ifPresent(this::configureBindings);
        launcher.ifPresent(this::configureBindings);
        intake.ifPresent(this::configureBindings);
        hopper.ifPresent(this::configureBindings);
        drivetrain.ifPresent(this::configureBindings);
    }

    private void configureBindings(CommandSwerveDrivetrain drivetrain)
    {

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() -> drive
                // Drive forward with negative Y (forward)
                .withVelocityX(-driverGamepad.getLeftY() * MaxSpeed)
                // Drive left with negative X (left)
                .withVelocityY(-driverGamepad.getLeftX() * MaxSpeed)
                // Drive counterclockwise with negative X (left)
                .withRotationalRate(-driverGamepad.getRightX() * MaxAngularRate)));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        driverGamepad.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driverGamepad.b().whileTrue(drivetrain
            .applyRequest(
                () -> point.withModuleDirection(new Rotation2d(-driverGamepad.getLeftY(), -driverGamepad.getLeftX()))));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driverGamepad.back().and(driverGamepad.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverGamepad.back().and(driverGamepad.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverGamepad.start().and(driverGamepad.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverGamepad.start().and(driverGamepad.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        driverGamepad.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        PathPlannerAuto auto = new PathPlannerAuto("testAuto");
        Pose2d startingPose = auto.getStartingPose();
        driverGamepad.povLeft().onTrue(this.pathfindToPose(startingPose, 0.0, false).andThen(auto));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    // TODO: the following bindings are designed for testing and need to changed for the final control scheme.
    // SCORE FUEL: x-> deploy y-> intake, b-> spinUp shooters, a-> convey, right joystick press-> feed (fuel shoots)
    // CLIMB: pov up-> extend, pov down-> climb, pov left-> stow
    // DUMP FUEL: left trigger-> clear launcher, left bumper-> clear hopper, left stick-> eject from intake
    // COLLAPSE HOPPER: start-> partial deploy, back-> stow
    // IDLE OR STOP SHOOTER: right bumper-> stop shooter, right trigger-> idle shooter

    private void configureBindings(Climber climber)
    {
        codriverGamepad.povLeft().onTrue(climber.stowCommand());
        codriverGamepad.povUp().onTrue(climber.extendCommand());
        codriverGamepad.povDown().onTrue(climber.climbCommand());
        codriverGamepad.povRight().onTrue(climber.toggleRatchetCommand());
    }

    private void configureBindings(Launcher launcher)
    {
        codriverGamepad.rightStick().whileTrue(launcher.feedThenStopCommand());
        codriverGamepad.leftTrigger().whileTrue(launcher.clearThenStopCommand());
        codriverGamepad.b().onTrue(launcher.spinUpShootersCommand());
        codriverGamepad.rightBumper().onTrue(launcher.stopShootersCommand());
        codriverGamepad.rightTrigger().onTrue(launcher.idleShootersCommand());
    }

    private void configureBindings(Intake intake)
    {
        codriverGamepad.y().whileTrue(intake.intakeThenStopCommand());
        codriverGamepad.leftStick().whileTrue(intake.ejectThenStopCommand());
        codriverGamepad.x().onTrue(intake.deployCommand());
        codriverGamepad.start().onTrue(intake.partialDeployCommand());
        codriverGamepad.back().onTrue(intake.stowCommand());
    }

    private void configureBindings(Hopper hopper)
    {
        codriverGamepad.a().whileTrue(hopper.feedThenStopCommand());
        codriverGamepad.leftBumper().whileTrue(hopper.clearThenStopCommand());
    }

    public Command getAutonomousCommand()
    {
        var drivetrain = this.drivetrain
            .orElseThrow(() -> new IllegalStateException("Drivetrain subsystem is required for autonomous"));
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            // Then slowly drive forward (away from us) for 5 seconds.
            drivetrain.applyRequest(() -> drive.withVelocityX(0.5)
                .withVelocityY(0)
                .withRotationalRate(0))
                .withTimeout(5.0),
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle));
    }

    // Load the path we want to pathfind to and follow
    public Command pathfindThenFollowPath(String pathName) throws Exception
    {
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName); // checked exception

        // Create the constraints to use while pathfinding. The constraints defined in the path will only be used for
        // the path.
        PathConstraints constraints = new PathConstraints(
            PathPlannerConstants.MAX_VELOCITY_MPS, PathPlannerConstants.MAX_ACCELERATION_MPS_SQ,
            PathPlannerConstants.MAX_ANGULAR_VELOCITY_RPS, PathPlannerConstants.MAX_ANGULAR_ACCELERATION_RPS);

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
            path,
            constraints);
        return pathfindingCommand;
    }

    public void pathFindToMultiPath(List<String> pathName) throws Exception
    {
        PathConstraints constraints = new PathConstraints(
            PathPlannerConstants.MAX_VELOCITY_MPS, PathPlannerConstants.MAX_ACCELERATION_MPS_SQ,
            PathPlannerConstants.MAX_ANGULAR_VELOCITY_RPS, PathPlannerConstants.MAX_ANGULAR_ACCELERATION_RPS);

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        var pathIterator = pathName.iterator();

        PathPlannerPath path = PathPlannerPath.fromPathFile(pathIterator.next()); // checked exception
        Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
            path,
            constraints);
        Command lastCommand = pathfindingCommand;

        while (pathIterator.hasNext())
        {
            path = PathPlannerPath.fromPathFile(pathIterator.next());
            pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
                path,
                constraints);
            lastCommand = lastCommand.andThen(pathfindingCommand);
        }

        CommandScheduler.getInstance().schedule(lastCommand);
    }

    public Command pathfindToPose(Pose2d point, Double endVelocity, boolean blueAlliance)
    {
        // Creates a command to pathfind to the given pose

        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
            5.0, 4.0,
            Units.degreesToRadians(540), Units.degreesToRadians(-180));
        Command pathfindingCommand;
        if (blueAlliance == false)
        {
            pathfindingCommand = AutoBuilder.pathfindToPoseFlipped(
                point,
                constraints,
                endVelocity // Goal end velocity in meters/sec
            );
        }
        else
        {
            // Since AutoBuilder is configured, we can use it to build pathfinding commands
            // flipped will reflect accross from where pathplanner says the path will start
            pathfindingCommand = AutoBuilder.pathfindToPose(
                point,
                constraints,
                endVelocity // Goal end velocity in meters/sec
            );
        }
        return pathfindingCommand;
    }

    public void pathFindToMultiPose(List<Pose2d> points)
    {

        // var cmd = pathfindToPose(points.get(0), 10.0);
        var pointsIterator = points.iterator();
        Command cmd = pathfindToPose(pointsIterator.next(), 5.0, false);
        Command lastCommand = cmd;
        while (pointsIterator.hasNext())
        {
            lastCommand = lastCommand.andThen(pathfindToPose(pointsIterator.next(), 10.0, false));
        }

        CommandScheduler.getInstance().schedule(lastCommand);
    }

    public void updateVisionEstimate()
    {
        if (this.drivetrain.isEmpty())
        {
            return;
        }
        var drivetrain = this.drivetrain.orElseThrow(
            () -> new IllegalStateException("Drivetrain subsystem is required to update vision pose estimation"));
        // updatePoseEstimation function assigns estimations to the
        // drive train.
        this.limelightVision.updatePoseEstimation(drivetrain);
    }

    public Command getTestCommand()
    {
        var group = new SequentialCommandGroup();

        // Wait for startup messages to be logged to driver station console:
        group.addCommands(new WaitCommand(5));
        if (drivetrain.isPresent())
        {
            group.addCommands(new TestDrivetrain(drivetrain.get()));
        }

        if (intake.isPresent())
        {
            group.addCommands(new TestIntake(intake.get()));
        }

        if (hopper.isPresent())
        {
            group.addCommands(new TestHopper(hopper.get()));
        }

        if (launcher.isPresent())
        {
            group.addCommands(new TestLauncher(launcher.get()));
        }

        if (climber.isPresent())
        {
            group.addCommands(new TestClimber(climber.get()));
        }

        return group;
    }

    private void configureDashboard()
    {
        // Climber:
        SmartDashboard.putNumber(DashboardConstants.CLIMBER_CLIMB_KEY, ClimberConstants.CLIMBED_POSITION);
        SmartDashboard.putNumber(DashboardConstants.CLIMBER_EXTEND_KEY, ClimberConstants.EXTENDED_POSITION);
        SmartDashboard.putNumber(DashboardConstants.CLIMBER_STOW_KEY, ClimberConstants.STOWED_POSITION);

        // Hopper:
        SmartDashboard.putNumber(DashboardConstants.CONVEYOR_FEED_KEY, HopperConstants.FEED_SPEED);
        SmartDashboard.putNumber(DashboardConstants.CONVEYOR_CLEAR_KEY, HopperConstants.CLEAR_SPEED);

        // Intake:
        SmartDashboard.putNumber(DashboardConstants.INTAKE_SPEED_KEY, IntakeConstants.INTAKE_SPEED);
        SmartDashboard.putNumber(DashboardConstants.EJECT_SPEED_KEY, IntakeConstants.EJECT_SPEED);
        SmartDashboard.putNumber(DashboardConstants.PIVOT_SPEED_KEY, IntakeConstants.PIVOT_SPEED);
        SmartDashboard.putNumber(DashboardConstants.DEPLOY_KEY, IntakeConstants.DEPLOYED_POSITION);
        SmartDashboard.putNumber(DashboardConstants.PARTIALLY_DEPLOY_KEY, IntakeConstants.PARTIALLY_DEPLOYED_POSITION);
        SmartDashboard.putNumber(DashboardConstants.STOW_KEY, IntakeConstants.STOW_POSITION);
        SmartDashboard.putNumber(DashboardConstants.PIVOT_TOLERANCE_KEY, IntakeConstants.DEPLOY_TOLERANCE);

        // Launcher:
        SmartDashboard.putNumber(DashboardConstants.LAUNCHER_SHOOTING_KEY, LauncherConstants.SHOOTING_VELOCITY_RPM);
        SmartDashboard.putNumber(DashboardConstants.LAUNCHER_IDLE_KEY, LauncherConstants.IDLE_VELOCITY_RPM);
        SmartDashboard.putNumber(DashboardConstants.LAUNCHER_FEED_KEY, LauncherConstants.FEED_SPEED);
        SmartDashboard.putNumber(DashboardConstants.LAUNCHER_CLEAR_KEY, LauncherConstants.CLEAR_SPEED);
    }
}
