// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.List;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.ArmMovement;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.Positions;
import frc.robot.util.JoystickToHeading;

public class RobotContainer {

    private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    /*
     * Swerve requests
     */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.FieldCentricFacingAngle driveFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final JoystickToHeading joystickToHeading = new JoystickToHeading();
    private final List<Rotation2d> headings = List.of(
            Rotation2d.kZero,
            Rotation2d.fromDegrees(-60),
            Rotation2d.fromDegrees(-120),
            Rotation2d.k180deg,
            Rotation2d.fromDegrees(120),
            Rotation2d.fromDegrees(60));

    private final SwerveRequest.Idle idle = new SwerveRequest.Idle();
    private final Telemetry logger = new Telemetry(MaxSpeed);

    /*
     * Controllers, subsystems, choosers
     */
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();
    private final SendableChooser<EventLoop> controllerModeChooser = new SendableChooser<>();

    private final EventLoop singlePlayer = new EventLoop();
    private final EventLoop twoPlayer = new EventLoop();
    private final CommandXboxController driverJoystick = new CommandXboxController(0);
    private final CommandXboxController operatorPanel = new CommandXboxController(1);

    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final Arm arm = new Arm();

    public RobotContainer() {
        configureSinglePlayerBindings();
        configureTwoPlayerBindings();
        changeEventLoop(singlePlayer);
        configureNamedCommands();
        configureAutoCommands();
        configureControlLoopChooser();
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

    public BooleanSupplier isEventLoopScheduled(EventLoop loop) {
        return () -> CommandScheduler.getInstance().getActiveButtonLoop().equals(loop);
    }

    public void changeEventLoop(EventLoop loop) {
        CommandScheduler.getInstance().setActiveButtonLoop(loop);
    }

    private void configureControlLoopChooser() {
        controllerModeChooser.setDefaultOption("Single player", singlePlayer);
        controllerModeChooser.addOption("Two player", twoPlayer);

        controllerModeChooser.onChange(this::changeEventLoop);
    }

    private void configureSinglePlayerBindings() {
        configureCommonBindings(singlePlayer);

       driverJoystick.b(singlePlayer).onTrue(ArmMovement.cmdSetElevatorPosition(Positions.Home));
       driverJoystick.rightBumper(singlePlayer).onTrue(ArmMovement.cmdSetElevatorPosition(Positions.FreeRangeMode));
       driverJoystick.leftBumper(singlePlayer).onTrue(ArmMovement.cmdSetElevatorPosition(Positions.ScoreCoral));
       driverJoystick.leftTrigger().onTrue(ArmMovement.cmdSetElevatorPosition(Positions.Intake));
       driverJoystick.rightTrigger().onTrue(ArmMovement.cmdSetElevatorPosition(Positions.Shoot));

        new ArmMovement(arm, Positions.Home);
        new ArmMovement(arm, Positions.Climb);
        new ArmMovement(arm, Positions.Intake);
        new ArmMovement(arm, Positions.ScoreCoral);
        new ArmMovement(arm, Positions.ClimbPrep);
    }

    private void configureTwoPlayerBindings() {
        configureCommonBindings(twoPlayer);

        new ArmMovement(arm, Positions.Home);
        new ArmMovement(arm, Positions.Climb);
        new ArmMovement(arm, Positions.Intake);
        new ArmMovement(arm, Positions.ScoreCoral);
        new ArmMovement(arm, Positions.ClimbPrep);
    }

    private void configureCommonBindings(EventLoop loop) {
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> driveFacingAngle.withVelocityX(-driverJoystick.getLeftY() * MaxSpeed)
                        .withVelocityY(-driverJoystick.getLeftX() * MaxSpeed)
                        .withTargetDirection(
                                joystickToHeading.determineHeading(
                                        -driverJoystick.getRightX(),
                                        -driverJoystick.getRightY()))
                        .withHeadingPID(10, 0, 0)).beforeStarting(
                                () -> joystickToHeading.setTargetHeading(drivetrain.getState().Pose.getRotation()
                                        .rotateBy(drivetrain.getOperatorForwardDirection()))));

        driverJoystick.rightTrigger(0.5, loop)
                .whileTrue(drivetrain.applyRequest(
                        () -> driveFacingAngle.withVelocityX(-driverJoystick.getLeftY() * MaxSpeed)
                                .withVelocityY(-driverJoystick.getLeftX() * MaxSpeed)
                                .withTargetDirection(
                                        joystickToHeading.determineSnapHeading(
                                                -driverJoystick.getRightX(),
                                                -driverJoystick.getRightY(),
                                                30.0,
                                                headings))
                                .withHeadingPID(10, 0, 0))
                        .beforeStarting(
                                () -> joystickToHeading.setTargetHeading(drivetrain.getState().Pose.getRotation()
                                        .rotateBy(drivetrain.getOperatorForwardDirection()))));
                                        
        driverJoystick.leftTrigger(0.5, loop).whileTrue(
                drivetrain.applyRequest(() -> drive.withVelocityX(-driverJoystick.getLeftY() * MaxSpeed)
                        .withVelocityY(-driverJoystick.getLeftX() * MaxSpeed)
                        .withRotationalRate(-driverJoystick.getRightX() * MaxAngularRate)));

        RobotModeTriggers.disabled()
                .whileTrue(drivetrain.applyRequest(() -> idle)
                        .ignoringDisable(true));

        driverJoystick.start(loop).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
    }

    private void addPathAuto(String name, String pathName) {
        try {
            autoChooser.addOption(name, new PathPlannerAuto(pathName));
        } catch (Exception e) {
            // Exceptions are now caught in the PathPlannerAuto constructor and this should
            // never run. Leaving it in place to catch any edge cases.
        }
    }

    private void configureAutoCommands() {
        /*
         * Do nothing as default is a human safety condition, this should always be the
         * default
         */
        autoChooser.setDefaultOption("Do nothing", new WaitCommand(15));
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureNamedCommands() {

    }

}
