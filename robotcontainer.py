from __future__ import annotations
import math

import commands2
import wpimath
import wpilib
import typing

from commands2 import cmd, InstantCommand, RunCommand, WaitCommand, ParallelCommandGroup, ParallelDeadlineGroup
from commands2.button import CommandGenericHID
from wpilib import XboxController
from wpimath.controller import PIDController, ProfiledPIDControllerRadians, HolonomicDriveController
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator

from constants import AutoConstants, DriveConstants, OIConstants
from subsystems.drivesubsystem import DriveSubsystem, BadSimPhysics
from subsystems.elevator import Elevator
from subsystems.intake import Intake
from subsystems.limelight_camera import LimelightCamera
from subsystems.singleIntake import SingleIntake

from constants import IntakeConstants, LiftConstants

from commands.reset_xy import ResetXY, ResetSwerveFront
from commands.aimtodirection import AimToDirection
from commands.approach import ApproachTag
from commands.arcadedrive import ArcadeDrive
from commands.elevatorposition import SetElevatorPosition
from commands.findobject import FindObject
from commands.followobject import FollowObject
from commands.gotopoint import GoToPoint
from commands.holonomicdrive import HolonomicDrive
from commands.intakecommands import Intake, IntakeGamepiece, IntakeEjectGamepieceBackward, IntakeFeedGamepieceForward
from commands.jerky_trajectory import JerkyTrajectory
from commands.setcamerapipeline import SetCameraPipeline
from commands.swervetopoint import SwerveToPoint, SwerveToSide, SwerveMove
from commands.trajectory_picker import TrajectoryPicker


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self, robot) -> None:
        # The robot's subsystems
        self.robotDrive = DriveSubsystem()
        if commands2.TimedCommandRobot.isSimulation():
            self.robotDrive.simPhysics = BadSimPhysics(self.robotDrive, robot)
        self.frontLimelight = LimelightCamera("frontLimelight")
        self.backLimelight = LimelightCamera("backLimelight")
        self.frontLimelight.setPiPMode(2)
        self.intake = Intake(leaderCanID=IntakeConstants.kLeadIntake, followerCanID=IntakeConstants.kFollowIntake,
                             leaderInverted=False, followerInverted=True, rangeFinder=None)
        self.singleIntake = SingleIntake(intakeCanID=IntakeConstants.kIntake)
        self.elevator = Elevator(leadMotorCANId=LiftConstants.kLeadLift, followMotorCANId=LiftConstants.kFollowLift,
                                 presetSwitchPositions=(0.1, 4.5, 12.39))

        # The driver's controller
        self.driverController = CommandGenericHID(OIConstants.kDriverControllerPort)
        self.operatorController = CommandGenericHID(OIConstants.kOperatorControllerPort)

        # Configure the button bindings and autos
        self.configureButtonBindings()
        self.configureAutos()

        # Configure default command for driving using sticks
        from commands.holonomicdrive import HolonomicDrive
        self.robotDrive.setDefaultCommand(
            HolonomicDrive(
                self.robotDrive,
                forwardSpeed=lambda: self.driverController.getRawAxis(XboxController.Axis.kLeftY),
                leftSpeed=lambda: self.driverController.getRawAxis(XboxController.Axis.kLeftX),
                rotationSpeed=lambda: -self.driverController.getRawAxis(XboxController.Axis.kRightX),
                deadband=OIConstants.kDriveDeadband,
                fieldRelative=True,
                rateLimit=True,
                square=True,
            )
        )
        self.elevator.setDefaultCommand(
            commands2.RunCommand(
                lambda: self.elevator.drive(self.operatorController.getRawAxis(XboxController.Axis.kLeftY)),
                self.elevator)
        )

    def configureButtonBindings(self) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """

        def swerveSide(metersToTheLeft: float, metersBackwards: float, drivetrain: DriveSubsystem, speed=1.0,
                       heading=None):
            """
            metersToTheLeft: How many meters you want to go to the left? Negative numbers for right.
            metersBackwards: How many meters you want to go backwards? Negative numbers for forward.
            drivetrain: The DriveSubsystem you'd like to use.
            speed: The speed you'd like the robot to swerve at.
            heading: The heading you'd like the robot to look at after swerving.
            """
            return SwerveMove(metersToTheLeft=metersToTheLeft, metersBackwards=metersBackwards, drivetrain=drivetrain,
                              speed=speed,
                              heading=heading)  # By Jonas: I had to do this because when I used SwerveMove twice

        # it would return errors.
        def roundHeading():
            """
            Returns the current heading of the robot to the nearest 60.
            """
            angle = self.robotDrive.getHeading().degrees()
            result = 60 * round(angle / 60)
            print(f"Rounded heading to {result} and angle is {angle}")
            return result

        # Driver Controller
        povUpDriverButton = self.driverController.pov(0)
        povUpDriverButton.onTrue(ResetXY(x=0.0, y=0.0, headingDegrees=0.0, drivetrain=self.robotDrive))
        povUpDriverButton.whileTrue(RunCommand(self.robotDrive.setX, self.robotDrive))

        povDownDriverButton = self.driverController.pov(180)
        povDownDriverButton.onTrue(ResetSwerveFront(self.robotDrive))

        yButtonDriver = self.driverController.button(XboxController.Button.kY)
        setPipeline0 = SetCameraPipeline(camera=self.frontLimelight, pipelineIndex=0)
        approachTagLeft = ApproachTag(camera=self.frontLimelight, drivetrain=self.robotDrive, speed=1.0,
                                          pushForwardSeconds=None,
                                          specificHeadingDegrees=roundHeading)
        swerveMiddle = swerveSide(metersToTheLeft=0.2, metersBackwards=0.0, drivetrain=self.robotDrive, speed=0.5)
        yButtonDriver.whileTrue(setPipeline0.andThen(approachTagLeft).andThen(swerveMiddle))

        bDriverController = self.driverController.button(XboxController.Button.kB)
        FeedUpper = IntakeFeedGamepieceForward(self.intake, motor1speed=-0.5).withTimeout(1.0)
        bDriverController.onTrue(FeedUpper)

        xDriverButton = self.driverController.button(XboxController.Button.kX)
        FeedLower = IntakeFeedGamepieceForward(self.intake, motor1speed=-0.5, motor2speed=-0.2).withTimeout(1.5)
        xDriverButton.onTrue(FeedLower)

        leftDriverBumper = self.driverController.button(XboxController.Button.kLeftBumper)
        setPipeline0 = SetCameraPipeline(camera=self.frontLimelight, pipelineIndex=0)
        approachTagLeft = ApproachTag(camera=self.frontLimelight, drivetrain=self.robotDrive, speed=1.0,
                                      pushForwardSeconds=None,
                                      specificHeadingDegrees=roundHeading)
        swerveToLeft = swerveSide(metersToTheLeft=0.32 , metersBackwards=0.0, drivetrain=self.robotDrive, speed=0.5)
        leftDriverBumper.whileTrue(setPipeline0.andThen(approachTagLeft).andThen(swerveToLeft))

        rightDriverBumper = self.driverController.button(XboxController.Button.kRightBumper)
        setPipeline0 = SetCameraPipeline(camera=self.frontLimelight, pipelineIndex=0)
        approachTagRight = ApproachTag(camera=self.frontLimelight, drivetrain=self.robotDrive, speed=1.0,
                                       pushForwardSeconds=None,
                                       specificHeadingDegrees=roundHeading)
        swerveToRight = swerveSide(metersToTheLeft=-0.0288, metersBackwards=0.0, drivetrain=self.robotDrive, speed=0.5)
        rightDriverBumper.whileTrue(setPipeline0.andThen(approachTagRight).andThen(swerveToRight))

        # Operator Controller
        leftOperatorBumper = self.operatorController.button(XboxController.Button.kLeftBumper)
        leftOperatorBumper.onTrue(InstantCommand(self.elevator.switchUp, self.elevator))

        rightOperatorBumper = self.operatorController.button(XboxController.Button.kRightBumper)
        rightOperatorBumper.onTrue(InstantCommand(self.elevator.switchDown, self.elevator))

        yOperatorButton = self.operatorController.button(XboxController.Button.kY)
        singleIntakeCmd = IntakeGamepiece(self.singleIntake, speed=-0.4)
        yOperatorButton.whileTrue(singleIntakeCmd)

        xOperatorButton = self.operatorController.button(XboxController.Button.kX)
        oppSingleIntake = IntakeGamepiece(self.singleIntake, speed=0.4)
        xOperatorButton.whileTrue(oppSingleIntake)

        aOperatorButton = self.operatorController.button(XboxController.Button.kA)
        intakeCmd = IntakeGamepiece(self.intake, speed=-0.2)
        aOperatorButton.whileTrue(intakeCmd)

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
        self.chosenAuto.addOption("Blue 1", self.getBlue1)
        self.chosenAuto.addOption("Blue 1 Station", self.getBlue1Station)
        self.chosenAuto.addOption("Blue 1 Low", self.getBlue1Low)
        self.chosenAuto.addOption("Blue 2", self.getBlue2)
        self.chosenAuto.addOption("Blue 3", self.getBlue3)
        self.chosenAuto.addOption("Blue 3 Low", self.getBlue3Low)
        self.chosenAuto.addOption("Red 1", self.getRed1)
        self.chosenAuto.addOption("Red 1 Low", self.getRed1Low)
        self.chosenAuto.addOption("Red 2", self.getRed2)
        self.chosenAuto.addOption("Red 3", self.getRed3)
        self.chosenAuto.addOption("Red 3 Low", self.getRed3Low)

        wpilib.SmartDashboard.putData("Chosen Auto", self.chosenAuto)

    def getBlue1(self):
        # Start to Reef
        startLocation = ResetXY(x=7.600, y=7.260, headingDegrees=180, drivetrain=self.robotDrive)
        setPipeline3 = SetCameraPipeline(camera=self.frontLimelight, pipelineIndex=3)
        aim145 = AimToDirection(degrees=-145, drivetrain=self.robotDrive)
        approachTag = ApproachTag(camera=self.frontLimelight, drivetrain=self.robotDrive, speed=1.0, pushForwardSeconds=None,
                                  specificHeadingDegrees=-120)
        level2 = SetElevatorPosition(elevator=self.elevator, position=4.5)
        getToReef = ParallelDeadlineGroup(approachTag, level2)
        score = IntakeFeedGamepieceForward(self.intake, motor1speed=-0.5).withTimeout(1.5)

        startToReef = startLocation.andThen(setPipeline3).andThen(aim145).andThen(getToReef).andThen(score)

        # Take Algae
        swerveLeft = SwerveMove(metersToTheLeft=0.2288, metersBackwards=0.0, drivetrain=self.robotDrive, speed=0.5)
        narwhalUp = SetElevatorPosition(elevator=self.elevator, position=8.826)
        narwhalOn = IntakeGamepiece(intake=self.singleIntake, speed=-0.4).withTimeout(2)
        backUp = SwerveToSide(metersToTheLeft=0.0, metersBackwards=0.5, drivetrain=self.robotDrive, speed=0.5)
        aim0 = AimToDirection(degrees=0, drivetrain=self.robotDrive)
        narwhalOff = IntakeGamepiece(intake=self.singleIntake, speed=0.4).withTimeout(2)
        level1 = SetElevatorPosition(elevator=self.elevator, position=0.1)

        narwhalWork = narwhalUp.alongWith(narwhalOn)
        algaeOff = aim0.andThen(narwhalOff).alongWith(level1)
        takeAlgae = swerveLeft.andThen(narwhalWork).andThen(backUp).andThen(algaeOff)

        command = startToReef.andThen(takeAlgae)
        return command

    def getBlue1Station(self):
        def startToReef():
            startLocation = ResetXY(x=7.600, y=7.260, headingDegrees=180, drivetrain=self.robotDrive)
            setPipeline3 = SetCameraPipeline(camera=self.frontLimelight, pipelineIndex=3)
            aim145 = AimToDirection(degrees=-145, drivetrain=self.robotDrive)
            approachTag = ApproachTag(camera=self.frontLimelight, drivetrain=self.robotDrive, speed=1.0,
                                      pushForwardSeconds=None,
                                      specificHeadingDegrees=-120)
            level2 = SetElevatorPosition(elevator=self.elevator, position=4.5)
            getToReef = ParallelDeadlineGroup(approachTag, level2)
            score = IntakeFeedGamepieceForward(self.intake, motor1speed=-0.5).withTimeout(1.5)

            return startLocation.andThen(setPipeline3).andThen(aim145).andThen(getToReef).andThen(score)

        def reefToStation():
            level1 = SetElevatorPosition(elevator=self.elevator, position=0.1)
            swerveRight = SwerveMove(drivetrain=self.robotDrive, speed=0.5, metersToTheLeft=-1.0, metersBackwards=0.5,
                                     heading=-30)
            setPipeline1 = SetCameraPipeline(camera=self.backLimelight, pipelineIndex=1)
            parallelCommand = ParallelDeadlineGroup(swerveRight, setPipeline1)
            approachFeeder = ApproachTag(camera=self.backLimelight, drivetrain=self.robotDrive, speed=1.0,
                                                       pushForwardSeconds=None, specificHeadingDegrees=-54)
            intake = IntakeFeedGamepieceForward(self.intake, motor1speed=-0.2)

            return level1.andThen(parallelCommand).andThen(approachFeeder).andThen(intake)

        def stationToScore():
            setPipeline3 = SetCameraPipeline(camera=self.frontLimelight, pipelineIndex=3)
            approachScore = ApproachTag(camera=self.frontLimelight, drivetrain=self.robotDrive, speed=1.0,
                                                       pushForwardSeconds=None, specificHeadingDegrees=-60)
            scoreLevel1 = IntakeFeedGamepieceForward(self.intake, motor1speed=-0.5, motor2speed=-0.2)

            return setPipeline3.andThen(approachScore).andThen(scoreLevel1)

        def scoreToStation():
            setPipeline1 = SetCameraPipeline(camera=self.backLimelight, pipelineIndex=1)
            approachStation = ApproachTag(camera=self.backLimelight, drivetrain=self.robotDrive, speed=1.0,
                                                       pushForwardSeconds=None, specificHeadingDegrees=-54)
            intake = IntakeFeedGamepieceForward(self.intake, motor1speed=-0.2)

            return setPipeline1.andThen(approachStation).andThen(intake)

        def repeatStationToScore():
            setPipeline3 = SetCameraPipeline(camera=self.frontLimelight, pipelineIndex=3)
            approachScore = ApproachTag(camera=self.frontLimelight, drivetrain=self.robotDrive, speed=1.0,
                                        pushForwardSeconds=None, specificHeadingDegrees=-60)
            swerveRight = SwerveMove(drivetrain=self.robotDrive, metersToTheLeft=-0.5, metersBackwards=0.0, speed=0.5)
            scoreLevel1 = IntakeFeedGamepieceForward(self.intake, motor1speed=-0.5, motor2speed=-0.2)

            return setPipeline3.andThen(swerveRight).andThen(approachScore).andThen(scoreLevel1)

        return startToReef().andThen(reefToStation()).andThen(stationToScore()).andThen(scoreToStation()).andThen(repeatStationToScore())

    def getBlue1Low(self):
        startLocation = ResetXY(x=7.600, y=7.260, headingDegrees=180, drivetrain=self.robotDrive)
        setPipeline3 = SetCameraPipeline(camera=self.frontLimelight, pipelineIndex=3)
        aimToTwoForty = AimToDirection(degrees=240, drivetrain=self.robotDrive)
        approachTag = ApproachTag(camera=self.frontLimelight, drivetrain=self.robotDrive, speed=1.0, pushForwardSeconds=None,
                                  specificHeadingDegrees=240)
        score = IntakeFeedGamepieceForward(self.intake, motor1speed=-0.5, motor2speed=-0.2).withTimeout(1.5)

        command = startLocation.andThen(setPipeline3).andThen(aimToTwoForty).andThen(approachTag).andThen(score)
        return command

    def getBlue2(self):
        startLocation = ResetXY(x=7.600, y=4.000, headingDegrees=180, drivetrain=self.robotDrive)
        setPipeline2 = SetCameraPipeline(camera=self.frontLimelight, pipelineIndex=2)
        approachTag = ApproachTag(camera=self.frontLimelight, drivetrain=self.robotDrive, speed=1.0, pushForwardSeconds=None,
                                  specificHeadingDegrees=180)
        score = IntakeFeedGamepieceForward(self.intake, motor1speed=-0.5, motor2speed=-0.2).withTimeout(1.5)

        command = startLocation.andThen(setPipeline2).andThen(approachTag).andThen(score)
        return command

    def getBlue3(self):
        #Start to Reef
        startLocation = ResetXY(x=7.600, y=0.795, headingDegrees=180, drivetrain=self.robotDrive)
        setPipeline3 = SetCameraPipeline(camera=self.frontLimelight, pipelineIndex=3)
        aim120 = AimToDirection(degrees=120, drivetrain=self.robotDrive)
        approachTag = ApproachTag(camera=self.frontLimelight, drivetrain=self.robotDrive, speed=1.0, pushForwardSeconds=None,
                                  specificHeadingDegrees=120)
        level2 = SetElevatorPosition(elevator=self.elevator, position=4.5)
        getToReef = ParallelDeadlineGroup(approachTag, level2)
        score = IntakeFeedGamepieceForward(self.intake, motor1speed=-0.5).withTimeout(1.5)

        startToReef = startLocation.andThen(setPipeline3).andThen(aim120).andThen(getToReef).andThen(score)

        #Take Algae
        swerveLeft = SwerveMove(metersToTheLeft=0.2288, metersBackwards=0.0, drivetrain=self.robotDrive, speed=0.5)
        narwhalUp = SetElevatorPosition(elevator=self.elevator, position=8.826)
        narwhalOn = IntakeGamepiece(intake=self.singleIntake, speed=-0.4).withTimeout(2)
        backUp = SwerveToSide(metersToTheLeft=0.0, metersBackwards=0.5, drivetrain=self.robotDrive, speed=0.5)
        aim0 = AimToDirection(degrees=0, drivetrain=self.robotDrive)
        narwhalOff = IntakeGamepiece(intake=self.singleIntake, speed=0.4).withTimeout(2)
        level1 = SetElevatorPosition(elevator=self.elevator, position=0.1)

        narwhalWork = narwhalUp.alongWith(narwhalOn)
        algaeOff = aim0.andThen(narwhalOff).alongWith(level1)
        takeAlgae = swerveLeft.andThen(narwhalWork).andThen(backUp).andThen(algaeOff)

        command = startToReef.andThen(takeAlgae)
        return command

    def getBlue3Low(self):
        startLocation = ResetXY(x=7.600, y=0.795, headingDegrees=180, drivetrain=self.robotDrive)
        setPipeline3 = SetCameraPipeline(camera=self.frontLimelight, pipelineIndex=3)
        aimToOneTwenty = AimToDirection(degrees=120, drivetrain=self.robotDrive)
        approachTag = ApproachTag(camera=self.frontLimelight, drivetrain=self.robotDrive, speed=1.0, pushForwardSeconds=None,
                                  specificHeadingDegrees=120)
        score = IntakeFeedGamepieceForward(self.intake, motor1speed=-0.5, motor2speed=-0.2).withTimeout(1.5)

        command = startLocation.andThen(setPipeline3).andThen(aimToOneTwenty).andThen(approachTag).andThen(score)
        return command

    def getRed1(self):
        #Start to Reef
        startLocation = ResetXY(x=10.000, y=0.795, headingDegrees=0, drivetrain=self.robotDrive)
        setPipeline3 = SetCameraPipeline(camera=self.frontLimelight, pipelineIndex=3)
        aim40 = AimToDirection(degrees=40, drivetrain=self.robotDrive)
        approachTag = ApproachTag(camera=self.frontLimelight, drivetrain=self.robotDrive, speed=1.0, pushForwardSeconds=None,
                                  specificHeadingDegrees=60)
        level2 = SetElevatorPosition(elevator=self.elevator, position=4.5)
        getToReef = ParallelDeadlineGroup(approachTag, level2)
        score = IntakeFeedGamepieceForward(self.intake, motor1speed=-0.5).withTimeout(1.5)

        startToReef = startLocation.andThen(setPipeline3).andThen(aim40).andThen(getToReef).andThen(score)

        #Take Algae
        swerveLeft = SwerveMove(metersToTheLeft=0.2288, metersBackwards=0.0, drivetrain=self.robotDrive, speed=0.5)
        narwhalUp = SetElevatorPosition(elevator=self.elevator, position=8.826)
        narwhalOn = IntakeGamepiece(intake=self.singleIntake, speed=-0.4).withTimeout(2)
        backUp = SwerveToSide(metersToTheLeft=0.0, metersBackwards=0.5, drivetrain=self.robotDrive, speed=0.5)
        aim180 = AimToDirection(degrees=180, drivetrain=self.robotDrive)
        narwhalOff = IntakeGamepiece(intake=self.singleIntake, speed=0.4).withTimeout(2)
        level1 = SetElevatorPosition(elevator=self.elevator, position=0.1)

        narwhalWork = narwhalUp.alongWith(narwhalOn)
        algaeOff = aim180.andThen(narwhalOff).alongWith(level1)
        takeAlgae = swerveLeft.andThen(narwhalWork).andThen(backUp).andThen(algaeOff)

        command = startToReef.andThen(takeAlgae)
        return command

    def getRed1Low(self):
        startLocation = ResetXY(x=10.000, y=0.795, headingDegrees=0, drivetrain=self.robotDrive)
        setPipeline3 = SetCameraPipeline(camera=self.frontLimelight, pipelineIndex=3)
        aimToSixty = AimToDirection(degrees=60, drivetrain=self.robotDrive)
        approachTag = ApproachTag(camera=self.frontLimelight, drivetrain=self.robotDrive, speed=1.0, pushForwardSeconds=None,
                                  specificHeadingDegrees=60)
        score = IntakeFeedGamepieceForward(self.intake, motor1speed=-0.5, motor2speed=-0.2).withTimeout(1.5)

        command = startLocation.andThen(setPipeline3).andThen(aimToSixty).andThen(approachTag).andThen(score)
        return command

    def getRed2(self):
        startLocation = ResetXY(x=10.000, y=4.000, headingDegrees=0, drivetrain=self.robotDrive)
        setPipeline2 = SetCameraPipeline(camera=self.frontLimelight, pipelineIndex=2)
        approachTag = ApproachTag(camera=self.frontLimelight, drivetrain=self.robotDrive, speed=1.0, pushForwardSeconds=None,
                                  specificHeadingDegrees=0)
        score = IntakeFeedGamepieceForward(self.intake, motor1speed=-0.5, motor2speed=-0.2).withTimeout(1.5)

        command = startLocation.andThen(setPipeline2).andThen(approachTag).andThen(score)
        return command

    def getRed3(self):
        #Start to Reef
        startLocation = ResetXY(x=10.000, y=7.276, headingDegrees=0, drivetrain=self.robotDrive)
        setPipeline3 = SetCameraPipeline(camera=self.frontLimelight, pipelineIndex=3)
        aim40 = AimToDirection(degrees=-40, drivetrain=self.robotDrive)
        approachTag = ApproachTag(camera=self.frontLimelight, drivetrain=self.robotDrive, speed=1.0, pushForwardSeconds=None,
                                  specificHeadingDegrees=-60)
        level2 = SetElevatorPosition(elevator=self.elevator, position=4.5)
        getToReef = ParallelDeadlineGroup(approachTag, level2)
        score = IntakeFeedGamepieceForward(self.intake, motor1speed=-0.5).withTimeout(1.5)

        startToReef = startLocation.andThen(setPipeline3).andThen(aim40).andThen(getToReef).andThen(score)

        #Take Algae
        swerveLeft = SwerveMove(metersToTheLeft=0.2288, metersBackwards=0.0, drivetrain=self.robotDrive, speed=0.5)
        narwhalUp = SetElevatorPosition(elevator=self.elevator, position=8.826)
        narwhalOn = IntakeGamepiece(intake=self.singleIntake, speed=-0.4).withTimeout(2)
        backUp = SwerveToSide(metersToTheLeft=0.0, metersBackwards=0.5, drivetrain=self.robotDrive, speed=0.5)
        aim180 = AimToDirection(degrees=180, drivetrain=self.robotDrive)
        narwhalOff = IntakeGamepiece(intake=self.singleIntake, speed=0.4).withTimeout(2)
        level1 = SetElevatorPosition(elevator=self.elevator, position=0.1)

        narwhalWork = narwhalUp.alongWith(narwhalOn)
        algaeOff = aim180.andThen(narwhalOff).alongWith(level1)
        takeAlgae = swerveLeft.andThen(narwhalWork).andThen(backUp).andThen(algaeOff)

        command = startToReef.andThen(takeAlgae)
        return command

    def getRed3Low(self):
        startLocation = ResetXY(x=10.000, y=7.276, headingDegrees=0, drivetrain=self.robotDrive)
        setPipeline3 = SetCameraPipeline(camera=self.frontLimelight, pipelineIndex=3)
        aimToOneTwenty = AimToDirection(degrees=-60, drivetrain=self.robotDrive)
        approachTag = ApproachTag(camera=self.frontLimelight, drivetrain=self.robotDrive, speed=1.0, pushForwardSeconds=None,
                                  specificHeadingDegrees=-60)
        score = IntakeFeedGamepieceForward(self.intake, motor1speed=-0.5, motor2speed=-0.2).withTimeout(1.5)

        command = startLocation.andThen(setPipeline3).andThen(aimToOneTwenty).andThen(approachTag).andThen(score)
        return command

    def getTestCommand(self) -> typing.Optional[commands2.Command]:
        """
        :returns: the command to run in test mode (to exercise all systems)
        """
        return None
