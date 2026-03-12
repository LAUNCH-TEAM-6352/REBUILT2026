// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.List;
import java.util.Optional;
import java.util.Set;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutomationConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DashboardConstants;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.MoveClimberWithGamepad;
import frc.robot.commands.MoveIntakePivotWithGamepad;
import frc.robot.commands.ScoreFuel;
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
import com.pathplanner.lib.commands.PathPlannerAuto;

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
    private final SwerveRequest.SwerveDriveBrake m_brakeRequest = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.FieldCentricFacingAngle faceAngle = new SwerveRequest.FieldCentricFacingAngle();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    // wrapper class to manage limelight cameras and get position estimates
    public final LimeLightVision limelightVision = new LimeLightVision(
        List.of("limelight-front", "limelight-br", "limelight-bl", "limelight-climber"));

    // Subsystems:
    private final Optional<Climber> climber;
    private final Optional<Launcher> launcher;
    private final Optional<Intake> intake;
    private final Optional<Hopper> hopper;
    private final Optional<CommandSwerveDrivetrain> drivetrain;

    // OI devices:
    private final CommandXboxController driverGamepad;
    private final CommandXboxController codriverGamepad;

    // Points for Paths/Automation
    Translation2d redHub = new Translation2d(11.9, 4);
    Translation2d blueHub = new Translation2d(4.65, 4);

    Translation2d topAllianceSideBump = new Translation2d(3.0, 5.6);
    Translation2d bottomAllianceSideBump = new Translation2d(3.0, 2.5);
    Translation2d topHubSideBump = new Translation2d(6.1, 5.6);
    Translation2d bottomHubSideBump = new Translation2d(6.1, 2.5);
    Translation2d topMiddleBump = new Translation2d(4.6, 5.6);
    Translation2d bottomMiddleBump = new Translation2d(4.6, 2.5);

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
        /*
         * NamedCommands.registerCommand("runIntake", intake.map(i -> i.intakeCommand()).orElse(Commands.none()));
         *
         * NamedCommands.registerCommand("stopShoot", stopAutoShootForAuto());
         *
         * NamedCommands.registerCommand("climb", autoClimb());
         *
         * NamedCommands.registerCommand("depotShootClimb",
         * Commands.sequence(intake.map(i -> i.runOnce(i::stop)).orElse(Commands.none()), autoShootCommandForAuto()));
         *
         * NamedCommands.registerCommand("humanShootClimb",
         * Commands.sequence(intake.map(i -> i.runOnce(i::stop)).orElse(Commands.none()), autoShootCommandForAuto()));
         *
         * NamedCommands.registerCommand("neutralShootClimb",
         * Commands.sequence(autoCrossBumpCommand(), autoShootCommandForAuto()));
         */
        if (launcher.isPresent())
        {
            NamedCommands.registerCommand("runShoot", launcher.get().spinUpShootersCommand());
        }
        if (intake.isPresent())
        {
            NamedCommands.registerCommand("runIntake", intake.get().intakeCommand());
        }
        if (climber.isPresent())
        {
            NamedCommands.registerCommand("runClimb", climber.get().climbCommand());
        }
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
        if (intake.isPresent() && hopper.isPresent() && launcher.isPresent())
        {
            configureBindings(intake.get(), hopper.get(), launcher.get());
        }
    }

    private void configureBindings(Intake intake, Hopper hopper, Launcher launcher)
    {
        driverGamepad.x().whileTrue(
            new ScoreFuel(launcher, hopper, intake).finallyDo(() ->
            {
                launcher.stopShooters();
                launcher.stopIndexer();
                hopper.stop();
            }));
    }

    private void configureBindings(CommandSwerveDrivetrain drivetrain)
    {
        faceAngle.HeadingController.setPID(5.0, 0.0, 0.0);

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
        /*
         * driverGamepad.back().and(driverGamepad.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
         * driverGamepad.back().and(driverGamepad.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
         * driverGamepad.start().and(driverGamepad.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
         * driverGamepad.start().and(driverGamepad.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
         */
        // Reset the field-centric heading on left bumper press.
        driverGamepad.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        /*
         * PathPlannerAuto auto = new PathPlannerAuto("testAuto");
         * Pose2d startingPose = auto.getStartingPose();
         * driverGamepad.povLeft().onTrue(this.pathfindToPose(startingPose, 0.0, false).andThen(auto));
         */
        /*
         * PathPlannerAuto neutralShootClimbPath = new PathPlannerAuto("neutralShootClimb");
         * Pose2d startingPoseNSCP = neutralShootClimbPath.getStartingPose();
         *
         * PathPlannerAuto humanShootClimb = new PathPlannerAuto("humanShootClimb");
         * Pose2d startingPoseHSC = humanShootClimb.getStartingPose();
         *
         * PathPlannerAuto depotShootClimb = new PathPlannerAuto("depotShootClimb");
         * Pose2d startingPoseDSC = depotShootClimb.getStartingPose();
         */
        PathPlannerAuto neutralShootClimbLeft = new PathPlannerAuto("neutralShootClimbLeft");
        Pose2d startingPoseNSCPL = neutralShootClimbLeft.getStartingPose();

        // PathPlannerAuto neutralShootClimbRight = new PathPlannerAuto("neutralShootClimbRight");
        // Pose2d startingPoseNSCPR = neutralShootClimbRight.getStartingPose();

        PathPlannerAuto humanShootClimbLeft = new PathPlannerAuto("humanShootClimbLeft");
        Pose2d startingPoseHSCL = humanShootClimbLeft.getStartingPose();

        PathPlannerAuto humanShootClimbRight = new PathPlannerAuto("humanShootClimbRight");
        Pose2d startingPoseHSCR = humanShootClimbRight.getStartingPose();

        PathPlannerAuto depotShootClimbRight = new PathPlannerAuto("depotShootClimbRight");
        Pose2d startingPoseDSCR = depotShootClimbRight.getStartingPose();

        PathPlannerAuto depotShootClimbLeft = new PathPlannerAuto("depotShootClimbLeft");
        Pose2d startingPoseDSCL = depotShootClimbLeft.getStartingPose();

        PathPlannerAuto testAutoShoot = new PathPlannerAuto("testAutoShoot");
        Pose2d startingPoseTest = testAutoShoot.getStartingPose();

        PathPlannerAuto testAutoClimb = new PathPlannerAuto("testAutoClimb");
        Pose2d startingPoseClimb = testAutoClimb.getStartingPose();

        drivetrain.registerTelemetry(logger::telemeterize);

        // THESE BINDS ARE JUST TESTING ONCE AGAIN THESE WILL CHANGE FOR THE FINAL CONTROL SCHEME
        driverGamepad
            .povUp().whileTrue(this.autoShootCommand());

        driverGamepad.povRight().whileTrue(this.autoFerry());

        driverGamepad.povLeft().whileTrue(this.autoCrossBumpCommand());

        driverGamepad.povDown().whileTrue(this.autoClimb());

        driverGamepad.leftStick().whileTrue(this.autoDeclimbCommand());

        driverGamepad.rightTrigger()
            .whileTrue(this.pathfindToPose(startingPoseNSCPL, 0.0).andThen(neutralShootClimbLeft));

        driverGamepad.leftTrigger()
            .whileTrue(this.pathfindToPose(startingPoseDSCR, 0.0).andThen(depotShootClimbRight));

        driverGamepad.rightStick()
            .whileTrue(this.pathfindToPose(startingPoseHSCL, 0.0).andThen(humanShootClimbLeft));

        driverGamepad.x()
            .whileTrue(this.pathfindToPose(startingPoseTest, 0.0).andThen(testAutoShoot));

        driverGamepad.back()
            .whileTrue(this.pathfindToPose(startingPoseClimb, 0.0).andThen(testAutoClimb));
    }

    // TODO: the following bindings are designed for testing and need to changed for the final control scheme.
    // SCORE FUEL: x-> deploy y-> intake, b-> spinUp shooters, a-> convey, right joystick press-> feed (fuel shoots)
    // CLIMB: pov up-> extend, pov down-> climb, pov left-> stow, pov right-> move with left stick up/down,
    // left stick left-> release ratchet, left stick right-> engage ratchet
    // DUMP FUEL: left trigger-> clear launcher, left bumper-> clear hopper, left stick-> eject from intake
    // COLLAPSE HOPPER: start-> partial deploy, back-> stow
    // IDLE OR STOP SHOOTER: right bumper-> stop shooter, right trigger-> idle shooter
    // MANUAL CONTROL OF INTAKE PIVIOT: right stick left->enable manual control, right stick forward/back-> move intake
    // pivot in/out

    private void configureBindings(Climber climber)
    {
        codriverGamepad.povLeft().onTrue(climber.stowCommand());
        codriverGamepad.povUp().onTrue(climber.extendCommand());
        codriverGamepad.povDown().onTrue(climber.climbCommand());
        codriverGamepad.povRight().onTrue(new MoveClimberWithGamepad(climber, codriverGamepad));
        new Trigger(() -> codriverGamepad.getLeftX() < -0.8).onTrue(climber.releaseRatchetCommand());
        new Trigger(() -> codriverGamepad.getLeftX() > 0.8).onTrue(climber.engageRatchetCommand());
    }

    private void configureBindings(Launcher launcher)
    {
        codriverGamepad.a().whileTrue(launcher.feedThenStopCommand());
        codriverGamepad.b().onTrue(launcher.spinUpShootersCommand());
        codriverGamepad.rightBumper().onTrue(launcher.stopShootersCommand());
        codriverGamepad.rightTrigger().onTrue(launcher.idleShootersCommand());
    }

    private void configureBindings(Intake intake)
    {
        codriverGamepad.y().whileTrue(intake.intakeThenStopCommand());
        codriverGamepad.x().onTrue(intake.deployCommand());
        codriverGamepad.start().onTrue(intake.partialDeployCommand());
        codriverGamepad.back().onTrue(intake.stowCommand());
        new Trigger(() -> codriverGamepad.getRightX() < -0.8)
            .onTrue(new MoveIntakePivotWithGamepad(intake, codriverGamepad));
    }

    private void configureBindings(Hopper hopper)
    {
        driverGamepad.a().whileTrue(hopper.feedThenStopCommand());
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

    public Command pathfindToPose(Pose2d point, Double endVelocity)
    {
        // Creates a command to pathfind to the given pose
        boolean blueAlliance = isBlueAlliance();

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
        Command cmd = pathfindToPose(pointsIterator.next(), 5.0);
        Command lastCommand = cmd;
        while (pointsIterator.hasNext())
        {
            lastCommand = lastCommand.andThen(pathfindToPose(pointsIterator.next(), 10.0));
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

    public Command autoShootCommand()
    {
        return Commands.sequence(

            Commands.runOnce(() -> goToShootPoint(
                SmartDashboard.getNumber(DashboardConstants.SHOOTING_POINT_RADIUS_KEY,
                    AutomationConstants.SHOOT_POINT_RADIUS_METERS))),

            Commands.parallel(
                drivetrain.map(dt -> dt.applyRequest(() -> m_brakeRequest))
                    .orElse(Commands.none()),
                intake.map(i -> i.intakeThenStopCommand())
                    .orElse(Commands.none()),
                launcher.map(l -> l.feedThenStopCommand())
                    .orElse(Commands.none()),
                hopper.map(h -> h.feedThenStopCommand())
                    .orElse(Commands.none())));
    }

    public Command autoShootCommandForAuto()
    {
        return Commands.sequence(

            Commands.runOnce(() -> goToShootPoint(
                SmartDashboard.getNumber(DashboardConstants.SHOOTING_POINT_RADIUS_KEY,
                    AutomationConstants.SHOOT_POINT_RADIUS_METERS))),
            launcher.map(l -> l.feedCommand())
                .orElse(Commands.none()),
            Commands.parallel(
                drivetrain.map(dt -> dt.applyRequest(() -> m_brakeRequest))
                    .orElse(Commands.none()),
                intake.map(i -> i.intakeCommand())
                    .orElse(Commands.none()),

                launcher.map(l -> l.spinUpShootersCommand())
                    .orElse(Commands.none()),
                hopper.map(h -> h.feedCommand())
                    .orElse(Commands.none())));
    }

    public Command stopAutoShootForAuto()
    {
        return Commands.parallel(
            intake.map(i -> i.runOnce(i::stop)).orElse(Commands.none()),
            launcher.map(l -> l.runOnce(l::stopAll)).orElse(Commands.none()),
            hopper.map(h -> h.runOnce(h::stop)).orElse(Commands.none()));
    }

    public Command autoFerry()
    {
        Rotation2d angle = Rotation2d.fromDegrees(0);
        if (isBlueAlliance())
        {
            angle = Rotation2d.fromDegrees(0);
        }
        else
        {
            angle = Rotation2d.fromDegrees(180);
        }

        return Commands.sequence(
            facePose2D(angle),
            Commands.parallel(
                intake.map(i -> i.intakeThenStopCommand())
                    .orElse(Commands.none()),
                launcher.map(l -> l.feedThenStopCommand())
                    .orElse(Commands.none()),
                hopper.map(h -> h.feedThenStopCommand())
                    .orElse(Commands.none())));
    }

    public Command autoCrossBumpCommand()
    {
        return Commands.defer(() ->
        {
            boolean isBlue = isBlueAlliance();
            double bumpX = isBlue ? 4.6 : 11.8;
            boolean onTopHalf = isBlue
                ? drivetrain.get().getPosition().getY() > 4.0
                : drivetrain.get().getPosition().getY() < 4.0;

            boolean onLeftSide = drivetrain.get().getPosition().getX() < bumpX;
            boolean goingToHub = (isBlue == onLeftSide);

            Translation2d middleBump = onTopHalf ? topMiddleBump : bottomMiddleBump;

            if (goingToHub)
            {
                Translation2d hubSide = onTopHalf ? topHubSideBump : bottomHubSideBump;
                return pathfindToPose(new Pose2d(middleBump, new Rotation2d(Math.PI / 4)), 5.0)
                    .andThen(pathfindToPose(new Pose2d(hubSide, new Rotation2d(0)), 0.0));
            }
            else
            {
                Translation2d allianceSide = onTopHalf ? topAllianceSideBump : bottomAllianceSideBump;
                return pathfindToPose(new Pose2d(middleBump, new Rotation2d(3 * Math.PI / 4)), 5.0)
                    .andThen(pathfindToPose(new Pose2d(allianceSide, new Rotation2d(Math.PI)), 0.0));
            }
        }, drivetrain.map(dt -> Set.of((Subsystem) dt)).orElse(Set.of()));
    }

    public Command autoClimb()
    {
        return Commands.sequence(
            pathfindToPose(new Pose2d(2.0, 3.75, Rotation2d.fromDegrees(180)), 0.0),
            climber.map(c -> c.releaseRatchetCommand()).orElse(Commands.none()),
            climber.map(c -> c.extendCommand()).orElse(Commands.none()),
            new WaitCommand(1),
            drivetrain
                .map(dt -> dt
                    .applyRequest(() -> drive.withVelocityX(-0.5 * MaxSpeed).withVelocityY(0).withRotationalRate(0)))
                .orElse(Commands.none()).withTimeout(0.5),
            climber.map(c -> c.climbCommand()).orElse(Commands.none()),
            climber.map(c -> c.engageRatchetCommand()).orElse(Commands.none()));
    }

    public Command autoDeclimbCommand()
    {
        return Commands.sequence(
            climber.map(c -> c.releaseRatchetCommand()).orElse(Commands.none()),
            climber.map(c -> c.extendCommand()).orElse(Commands.none()),
            drivetrain.map(
                dt -> dt.applyRequest(() -> drive.withVelocityX(0.5 * MaxSpeed).withVelocityY(0).withRotationalRate(0)))
                .orElse(Commands.none()).withTimeout(0.5),
            climber.map(c -> c.stowCommand()).orElse(Commands.none()),
            climber.map(c -> c.engageRatchetCommand()).orElse(Commands.none()));
    }

    public boolean isBlueAlliance()
    {
        Optional<Alliance> botAlliance = DriverStation.getAlliance();
        return botAlliance.isPresent() && botAlliance.get() == Alliance.Blue;
    }

    public Command facePose2D(Rotation2d angle)
    {
        return drivetrain.map(dt -> dt.applyRequest(() -> faceAngle
            .withVelocityX(-driverGamepad.getLeftY() * MaxSpeed)
            .withVelocityY(-driverGamepad.getLeftX() * MaxSpeed)
            .withTargetDirection(angle)))
            .orElse(Commands.none());
    }

    public Pose2d shootingCircle(double theta, double radius, boolean yesBlue)
    {
        double x = (radius * Math.cos(theta)) + blueHub.getX();
        double y = (radius * Math.sin(theta)) + blueHub.getY();
        Pose2d circlePos = new Pose2d(x, y, new Rotation2d(0));
        return circlePos;
    }

    public void goToShootPoint(double Radius)
    {
        Translation2d RobotPose = drivetrain.get().getPosition().getTranslation();
        boolean isBlue = isBlueAlliance();
        double lowerLim = (2 * Math.PI) / 3;
        double upperLim = (4 * Math.PI) / 3;
        double minRad = lowerLim;
        double minDistance = Double.MAX_VALUE;

        Translation2d searchPose = RobotPose;
        if (!isBlue)
        {
            searchPose = new Translation2d(16.54 - RobotPose.getX(), 8.21 - RobotPose.getY());
        }

        for (double i = lowerLim; i < upperLim; i += 0.01)
        {
            double dist = searchPose.getDistance(shootingCircle(i, Radius, isBlue).getTranslation());
            if (dist < minDistance)
            {
                minDistance = dist;
                minRad = i;
            }
        }

        Pose2d circlePos = shootingCircle(minRad, Radius, isBlue);

        Rotation2d facingRotation = Rotation2d.fromRadians(minRad + Math.PI);

        if (minDistance < 0.5)
        {
            facePose2D(facingRotation).withTimeout(0.5).schedule();
        }
        else
        {
            var cmd = pathfindToPose(new Pose2d(circlePos.getTranslation(), facingRotation), 0.0);
            CommandScheduler.getInstance().schedule(cmd);
        }
    }

    private void configureDashboard()
    {
        // Climber:
        SmartDashboard.putNumber(DashboardConstants.CLIMBER_CLIMB_KEY, ClimberConstants.CLIMBED_POSITION);
        SmartDashboard.putNumber(DashboardConstants.CLIMBER_EXTEND_KEY, ClimberConstants.EXTENDED_POSITION);
        SmartDashboard.putNumber(DashboardConstants.CLIMBER_STOW_KEY, ClimberConstants.STOWED_POSITION);

        // Hopper:
        SmartDashboard.putNumber(DashboardConstants.CONVEYOR_FEED_KEY, HopperConstants.FEED_SPEED);

        // Intake:
        SmartDashboard.putNumber(DashboardConstants.INTAKE_VELOCITY_KEY, IntakeConstants.INTAKE_VELOCITY_RPM);
        SmartDashboard.putNumber(DashboardConstants.DEPLOYED_KEY, IntakeConstants.DEPLOYED_POSITION.magnitude());
        SmartDashboard.putNumber(DashboardConstants.PARTIALLY_DEPLOYED_KEY,
            IntakeConstants.PARTIALLY_DEPLOYED_POSITION.magnitude());
        SmartDashboard.putNumber(DashboardConstants.STOWED_KEY, IntakeConstants.STOWED_POSITION.magnitude());
        SmartDashboard.putNumber(DashboardConstants.PIVOT_TOLERANCE_KEY, IntakeConstants.PIVOT_TOLERANCE_DEG);

        // Launcher:
        SmartDashboard.putNumber(DashboardConstants.LAUNCHER_SHOOTING_KEY, LauncherConstants.SHOOTING_VELOCITY_RPM);
        SmartDashboard.putNumber(DashboardConstants.LAUNCHER_IDLE_KEY, LauncherConstants.IDLE_VELOCITY_RPM);
        SmartDashboard.putNumber(DashboardConstants.LAUNCHER_FEED_KEY, LauncherConstants.FEED_VELOCITY_RPM);

        // Limelight:
        SmartDashboard.putNumber(DashboardConstants.LIMELIGHT_THROTTLE_DISABLED_KEY,
            Constants.LimeLightConstants.LIMELIGHT_THROTTLE_DISABLED);
        SmartDashboard.putNumber(DashboardConstants.LIMELIGHT_THROTTLE_ENABLED_KEY,
            Constants.LimeLightConstants.LIMELIGHT_THROTTLE_ENABLED);

        // Automation:
        SmartDashboard.putNumber(DashboardConstants.SHOOTING_POINT_RADIUS_KEY,
            AutomationConstants.SHOOT_POINT_RADIUS_METERS);
    }
}
