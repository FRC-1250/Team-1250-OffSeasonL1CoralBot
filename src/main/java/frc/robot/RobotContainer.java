// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.ArmMovement;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.Positions;

public class RobotContainer {
    public enum ReefHeading {
        AB(Rotation2d.kZero),
        CD(Rotation2d.fromDegrees(-60)),
        EF(Rotation2d.fromDegrees(-120)),
        HG(Rotation2d.k180deg),
        JI(Rotation2d.fromDegrees(120)),
        LK(Rotation2d.fromDegrees(60));

        private final Rotation2d rotation2d;

        private ReefHeading(Rotation2d rotation2d) {
            this.rotation2d = rotation2d;
        }

        public Rotation2d getRotation2d() {
            return rotation2d;
        }
    }

    private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

    private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.FieldCentricFacingAngle driveFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.Idle idle = new SwerveRequest.Idle();
    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final CommandXboxController joystick = new CommandXboxController(0);

    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final Arm arm = new Arm();

    private final double JOYSTICK_DEADBAND = 0.5;

    private final double HEADING_TOLERANCE_DEGREES = 30;

    private ReefHeading targetReefHeading = ReefHeading.HG;
    private ReefHeading previousReefHeading = ReefHeading.HG;

    private Rotation2d targetHeading = Rotation2d.kZero;
    private Rotation2d previousTargetHeading = Rotation2d.kZero;

    public RobotContainer() {
        configureBindings();
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

    private void configureBindings() {
        new ArmMovement(arm, Positions.Home);
        new ArmMovement(arm, Positions.Climb);
        new ArmMovement(arm, Positions.Intake);
        new ArmMovement(arm, Positions.ScoreCoral);
        new ArmMovement(arm, Positions.ClimbPrep);

        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> driveFacingAngle.withVelocityX(-joystick.getLeftY() * MaxSpeed)
                        .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                        .withTargetDirection(determineHeading())
                        .withHeadingPID(10, 0, 0)));

        joystick.rightTrigger().whileTrue(
                drivetrain.applyRequest(() -> driveFacingAngle.withVelocityX(-joystick.getLeftY() * MaxSpeed)
                        .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                        .withTargetDirection(determineReefZoneHeading())
                        .withHeadingPID(10, 0, 0)));

        joystick.leftTrigger().whileTrue(
                drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed)
                        .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                        .withRotationalRate(-joystick.getRightX() * MaxAngularRate)));

        RobotModeTriggers
                .disabled()
                .whileTrue(drivetrain.applyRequest(() -> idle)
                        .ignoringDisable(true));

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(
                () -> point.withModuleDirection(
                        new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private Rotation2d determineHeading() {
        double x = -joystick.getRightX();
        double y = -joystick.getRightY();

        previousTargetHeading = targetHeading;

        if (Math.hypot(x, y) > JOYSTICK_DEADBAND) {
            targetHeading = Rotation2d.fromRadians(Math.atan2(x, y));
            return targetHeading;
        } else {
            return previousTargetHeading;
        }
    }

    private Rotation2d determineReefZoneHeading() {
        double x = -joystick.getRightX();
        double y = -joystick.getRightY();

        // Store current target before possible update
        previousReefHeading = targetReefHeading;

        if (Math.hypot(x, y) > JOYSTICK_DEADBAND) {

            // Calculate the stick angle (forward = 0 degrees)
            Rotation2d stickAngle = Rotation2d.fromRadians(Math.atan2(x, y));

            ReefHeading closestHeading = targetReefHeading;
            double minAngularDifference = Double.POSITIVE_INFINITY;
            double difference = 0;

            // Find the closest heading
            for (ReefHeading heading : ReefHeading.values()) {
                difference = Math.abs(heading.getRotation2d().minus(stickAngle).getDegrees());

                if (difference < minAngularDifference) {
                    minAngularDifference = difference;
                    closestHeading = heading;
                }
            }

            if (minAngularDifference < HEADING_TOLERANCE_DEGREES) {
                targetReefHeading = closestHeading;
            }

            return targetReefHeading.getRotation2d();
        }
        return previousReefHeading.getRotation2d();
    }
}
