from __future__ import annotations
import math

import commands2
import wpimath
import wpilib
import typing
import rev

from commands2 import cmd, InstantCommand, RunCommand
from commands.intakecommands import IntakeGamepiece, IntakeFeedGamepieceForward
from commands2.button import JoystickButton, POVButton
from wpilib import XboxController
from wpimath.controller import PIDController, ProfiledPIDControllerRadians, HolonomicDriveController
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator
from navx import AHRS

from constants import AutoConstants, DriveConstants, OIConstants, LiftConstants, IntakeConstants
from subsystems.drivesubsystem import DriveSubsystem
from subsystems.limelight_camera import LimelightCamera
from subsystems.elevator import Elevator
from subsystems.intake import Intake
from subsystems.singleIntake import SingleIntake

from commands.reset_xy import ResetXY, ResetSwerveFront
from commands.aimtodirection import AimToDirection
from commands.findobject import FindObject
from commands.followobject import FollowObject
from commands.gotopoint import GoToPoint
from commands.jerky_trajectory import JerkyTrajectory
from commands.setcamerapipeline import SetCameraPipeline
from commands.swervetopoint import SwerveToPoint


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        # The robot's subsystems
        self.robotDrive = DriveSubsystem()
        self.camera = LimelightCamera("limelight")  # name of your camera goes in parentheses
        self.elevator = Elevator(leadMotorCANId=LiftConstants.kLeadLift, followMotorCANId=LiftConstants.kFollowLift,
                                 presetSwitchPositions=(0.1,4.533,13))
        self.intake = Intake(leaderCanID=IntakeConstants.kLeadIntake, leaderInverted=True,
                             followerCanID=IntakeConstants.kFollowIntake, followerInverted=False, rangeFinder=None)
        self.singleIntake = SingleIntake(intakeCanID=13, intakeInverted=True,)


        # The driver's controller
        self.driverController = wpilib.XboxController(OIConstants.kDriverControllerPort)
        self.operatorController = wpilib.XboxController(OIConstants.kOperatorControllerPort)

        # Configure the button bindings and autos
        self.configureButtonBindings()
        self.configureAutos()

        # Configure default commands
        self.robotDrive.setDefaultCommand(
            # The left stick controls translation of the robot.
            # Turning is controlled by the X axis of the right stick.
            commands2.RunCommand(
                lambda: self.robotDrive.drive(
                    -wpimath.applyDeadband(
                        self.driverController.getLeftY(), OIConstants.kDriveDeadband
                    ),
                    -wpimath.applyDeadband(
                        self.driverController.getLeftX(), OIConstants.kDriveDeadband
                    ),
                    -wpimath.applyDeadband(
                        self.driverController.getRightX(), OIConstants.kDriveDeadband
                    ),
                    True,
                    True,
                    square=True,
                ),
                self.robotDrive,
            )
        )
        self.elevator.setDefaultCommand(
            commands2.RunCommand(lambda: self.elevator.drive(self.operatorController.getLeftY()), self.elevator)
         )


    def configureButtonBindings(self) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """
        #Driver button bindings
        #Not used
        povUpDriverButton = POVButton(self.driverController, 0)  # 0 degrees for POV up
        povUpDriverButton.onTrue(ResetXY(x=0.0, y=0.0, headingDegrees=0.0, drivetrain=self.robotDrive))
        povUpDriverButton.whileTrue(RunCommand(self.robotDrive.setX, self.robotDrive))  # use the swerve X brake when "X" is pressed

        povDownDriverButton = POVButton(self.driverController, 180)
        povDownDriverButton.onTrue(ResetSwerveFront(self.robotDrive))
        
        #Used
            #X = Different speeds feed
            #Y = Same speed feed
            #B = Look + align with AprilTag
            #A =

        bDriverButton = JoystickButton(self.driverController, XboxController.Button.kB)
        intakeFeedFwdCmdUpper = IntakeFeedGamepieceForward(self.intake, motor1speed=0.5).withTimeout(1.0)
        bDriverButton.onTrue(intakeFeedFwdCmdUpper) #Intake same speed
        

        xDriverButton = JoystickButton(self.driverController, XboxController.Button.kX)
        intakeFeedFwdCmdLower = IntakeFeedGamepieceForward(self.intake, motor1speed=0.5, motor2speed=0.2).withTimeout(1.5)
        xDriverButton.onTrue(intakeFeedFwdCmdLower) #Intake different speeds
        
        #Operator button bindings
            #Left Bumper = Elevator Up
            #Right Bumper = Elevator Down
            #A = Intake
            #X =
            #B =
            #Y = Single Intake
        # left bumper and right bumper will move elevator between presetSwitchPositions (see above) 
        leftDriverBumper = JoystickButton(self.operatorController, XboxController.Button.kLeftBumper)
        leftDriverBumper.onTrue(InstantCommand(self.elevator.switchUp, self.elevator))
        rightDriverBumper = JoystickButton(self.operatorController, XboxController.Button.kRightBumper)
        rightDriverBumper.onTrue(InstantCommand(self.elevator.switchDown, self.elevator))

        aOperatorButton = JoystickButton(self.operatorController, XboxController.Button.kA)
        intakeCmd = IntakeGamepiece(self.intake,speed=0.2)
        aOperatorButton.whileTrue(intakeCmd) #Intake

        yOperatorButton =JoystickButton(self.operatorController, XboxController.Button.kY)
        singleIntakeCmd = IntakeGamepiece(self.singleIntake, speed=0.4)
        yOperatorButton.whileTrue(singleIntakeCmd)

        bOperatorButton =JoystickButton(self.operatorController, XboxController.Button.kB)
        singleIntakeCmd = IntakeGamepiece(self.singleIntake, speed=-0.4)
        bOperatorButton.whileTrue(singleIntakeCmd)

        # the "A" button will request elevator to go to a special position of 33.0 inches
        #aOperatorButton = JoystickButton(self.operatorController, XboxController.Button.kA)
        #aOperatorButton.onTrue(InstantCommand(lambda: self.elevator.setPositionGoal(33.0), self.elevator))

    def disablePIDSubsystems(self) -> None:
        """Disables all ProfiledPIDSubsystem and PIDSubsystem instances.
        This should be called on robot disable to prevent integral windup."""

    def getAutonomousCommand(self) -> commands2.Command:
        """
        :returns: the command to run in autonomous
        """
        command = self.chosenAuto.getSelected()
        return command()

    def configureAutos(self):
        self.chosenAuto = wpilib.SendableChooser()
        # you can also set the default option, if needed
        self.chosenAuto.setDefaultOption("trajectory example", self.getAutonomousTrajectoryExample)
        self.chosenAuto.addOption("left blue", self.getAutonomousLeftBlue)
        self.chosenAuto.addOption("left red", self.getAutonomousLeftRed)
        self.chosenAuto.addOption("middle blue", self.getAutonomousMiddleBlue)
        wpilib.SmartDashboard.putData("Chosen Auto", self.chosenAuto)

    def getAutonomousMiddleBlue(self):
        setStartPose = ResetXY(x=0.0, y=0.0, headingDegrees=0.0, drivetrain=self.robotDrive)



    def getAutonomousLeftBlue(self):
        setStartPose = ResetXY(x=0.783, y=6.686, headingDegrees=+60, drivetrain=self.robotDrive)
        driveForward = commands2.RunCommand(lambda: self.robotDrive.arcadeDrive(xSpeed=1.0, rot=0.0), self.robotDrive)
        stop = commands2.InstantCommand(lambda: self.robotDrive.arcadeDrive(0, 0))

        command = setStartPose.andThen(driveForward.withTimeout(1.0)).andThen(stop)
        return command

    def getAutonomousLeftRed(self):
        setStartPose = ResetXY(x=15.777, y=4.431, headingDegrees=-120, drivetrain=self.robotDrive)
        driveForward = commands2.RunCommand(lambda: self.robotDrive.arcadeDrive(xSpeed=1.0, rot=0.0), self.robotDrive)
        stop = commands2.InstantCommand(lambda: self.robotDrive.arcadeDrive(0, 0))

        command = setStartPose.andThen(driveForward.withTimeout(2.0)).andThen(stop)
        return command

    def getAutonomousTrajectoryExample(self) -> commands2.Command:
        # Create config for trajectory
        config = TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared,
        )
        # Add kinematics to ensure max speed is actually obeyed
        config.setKinematics(DriveConstants.kDriveKinematics)

        # An example trajectory to follow. All units in meters.
        exampleTrajectory = TrajectoryGenerator.generateTrajectory(
            # Start at the origin facing the +X direction
            Pose2d(0, 0, Rotation2d(0)),
            # Pass through these two interior waypoints, making an 's' curve path
            [Translation2d(0.5, 0.5), Translation2d(1, -0.5)],
            # End 1.5 meters straight ahead of where we started, facing forward
            Pose2d(1.5, 0, Rotation2d(0)),
            config,
        )

        thetaController = ProfiledPIDControllerRadians(
            AutoConstants.kPThetaController,
            0,
            0,
            AutoConstants.kThetaControllerConstraints,
        )
        thetaController.enableContinuousInput(-math.pi, math.pi)

        driveController = HolonomicDriveController(
            PIDController(AutoConstants.kPXController, 0, 0),
            PIDController(AutoConstants.kPXController, 0, 0),
            thetaController,
        )

        swerveControllerCommand = commands2.SwerveControllerCommand(
            exampleTrajectory,
            self.robotDrive.getPose,  # Functional interface to feed supplier
            DriveConstants.kDriveKinematics,
            driveController,
            self.robotDrive.setModuleStates,
            (self.robotDrive,),
        )

        # Reset odometry to the starting pose of the trajectory.
        self.robotDrive.resetOdometry(exampleTrajectory.initialPose())

        # Run path following command, then stop at the end.
        return swerveControllerCommand.andThen(
            cmd.run(
                lambda: self.robotDrive.drive(0, 0, 0, False, False),
                self.robotDrive,
            )
        )

    def getTestCommand(self) -> typing.Optional[commands2.Command]:
        """
        :returns: the command to run in test mode (to exercise all systems)
        """
        return None


class Gyro(wpilib.TimedRobot):
    def robotInit(self):
        self.timer = wpilib.Timer()

    def robotPeriodic(self):
        currentAngle = self.gyro.getAngle()
        wpilib.SmartDashboard.putNumber("Gyro Angle", currentAngle)