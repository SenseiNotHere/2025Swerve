from __future__ import annotationsimport mathimport commands2import wpimathimport wpilibimport typingfrom commands2 import cmd, InstantCommand, RunCommand, WaitCommandfrom commands2.button import CommandGenericHIDfrom rev import LimitSwitchConfigfrom wpilib import XboxControllerfrom wpimath.controller import PIDController, ProfiledPIDControllerRadians, HolonomicDriveControllerfrom wpimath.geometry import Pose2d, Rotation2d, Translation2dfrom wpimath.trajectory import TrajectoryConfig, TrajectoryGeneratorfrom commands.approach import ApproachTagfrom commands.intakecommands import IntakeFeedGamepieceForward, IntakeGamepiece, IntakeEjectGamepieceBackwardfrom constants import AutoConstants, DriveConstants, OIConstants, LiftConstants, IntakeConstantsfrom subsystems.drivesubsystem import DriveSubsystemfrom subsystems.elevator import Elevatorfrom subsystems.intake import Intakefrom subsystems.limelight_camera import LimelightCamerafrom subsystems.singleIntake import SingleIntakefrom commands.reset_xy import ResetXY, ResetSwerveFrontfrom commands.aimtodirection import AimToDirectionfrom commands.arcadedrive import ArcadeDrivefrom commands.elevatorposition import SetElevatorPositionfrom commands.findobject import FindObjectfrom commands.followobject import FollowObject, StopWhenfrom commands.gotopoint import GoToPointfrom commands.holonomicdrive import HolonomicDrivefrom commands.jerky_trajectory import JerkyTrajectoryfrom commands.reset_xy import ResetXYfrom commands.setcamerapipeline import SetCameraPipelinefrom commands.swervetopoint import SwerveToPoint, SwerveToSide, SwerveMoveimport commands.intakecommandsclass RobotContainer:    """    This class is where the bulk of the robot should be declared. Since Command-based is a    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including    subsystems, commands, and button mappings) should be declared here.    """    def __init__(self) -> None:        # The robot's subsystems        self.robotDrive = DriveSubsystem()        self.limelight = LimelightCamera("limelight")        self.limelight.setPiPMode(2)        self.intake = Intake(leaderCanID=IntakeConstants.kLeadIntake, followerCanID=IntakeConstants.kFollowIntake,                             leaderInverted=False, followerInverted=True, rangeFinder=None)        self.singleIntake = SingleIntake(intakeCanID=IntakeConstants.kIntake)        self.elevator = Elevator(leadMotorCANId=LiftConstants.kLeadLift, followMotorCANId=LiftConstants.kFollowLift,                                 presetSwitchPositions=(0.1, 4.5, 12.51))        # The driver's controller        self.driverController = CommandGenericHID(OIConstants.kDriverControllerPort)        self.operatorController = CommandGenericHID(OIConstants.kOperatorControllerPort)        # Configure the button bindings and autos        self.configureButtonBindings()        self.configureAutos()        # Configure default command for driving using sticks        self.robotDrive.setDefaultCommand(            HolonomicDrive(                self.robotDrive,                forwardSpeed=lambda: self.driverController.getRawAxis(XboxController.Axis.kLeftY),                leftSpeed=lambda: self.driverController.getRawAxis(XboxController.Axis.kLeftX),                rotationSpeed=lambda: -self.driverController.getRawAxis(XboxController.Axis.kRightX),                deadband=OIConstants.kDriveDeadband,                fieldRelative=True,                rateLimit=True,                square=True,            )        )        self.elevator.setDefaultCommand(            commands2.RunCommand(                lambda: self.elevator.drive(self.operatorController.getRawAxis(XboxController.Axis.kLeftY)),                self.elevator)        )    def configureButtonBindings(self) -> None:        """        Use this method to define your button->command mappings. Buttons can be created by        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),        and then passing it to a JoystickButton.        """        def swerveSide(metersToTheLeft: float, metersBackwards: float, drivetrain: DriveSubsystem, speed=1.0,                       heading=None):            return SwerveMove(metersToTheLeft=metersToTheLeft, metersBackwards=metersBackwards, drivetrain=drivetrain,                              speed=speed, heading=heading) #By Jonas: I had to do this because when I used SwerveMove twice        #it would return errors.        def roundHeading():            angle = self.robotDrive.getHeading().degrees()            result = 60 * round(angle / 60)            print(f"Rounded heading to {result} and angle is {angle}")            return result        # Driver Controller        povUpDriverButton = self.driverController.pov(0)        povUpDriverButton.onTrue(ResetXY(x=0.0, y=0.0, headingDegrees=0.0, drivetrain=self.robotDrive))        povUpDriverButton.whileTrue(RunCommand(self.robotDrive.setX, self.robotDrive))        povDownDriverButton = self.driverController.pov(180)        povDownDriverButton.onTrue(ResetSwerveFront(self.robotDrive))        bDriverController = self.driverController.button(XboxController.Button.kB)        FeedUpper = IntakeFeedGamepieceForward(self.intake, motor1speed=-0.5).withTimeout(1.0)        bDriverController.onTrue(FeedUpper)        xDriverButton = self.driverController.button(XboxController.Button.kX)        FeedLower = IntakeFeedGamepieceForward(self.intake, motor1speed=-0.5, motor2speed=-0.2).withTimeout(1.5)        xDriverButton.onTrue(FeedLower)        leftDriverBumper = self.driverController.button(XboxController.Button.kLeftBumper)        setPipeline0 = SetCameraPipeline(camera=self.limelight, pipelineIndex=0)        approachTagLeft = ApproachTag(camera=self.limelight, drivetrain=self.robotDrive, speed=1.0,                                       pushForwardSeconds=None,                                       specificHeadingDegrees=roundHeading)        swerveToLeft = swerveSide(metersToTheLeft=0.4, metersBackwards=-0.2, drivetrain=self.robotDrive, speed=0.5)        leftDriverBumper.whileTrue(setPipeline0.andThen(approachTagLeft).andThen(swerveToLeft))        rightDriverBumper = self.driverController.button(XboxController.Button.kRightBumper)        setPipeline0 = SetCameraPipeline(camera=self.limelight, pipelineIndex=0)        approachTagRight = ApproachTag(camera=self.limelight, drivetrain=self.robotDrive, speed=1.0, pushForwardSeconds=None,                                       specificHeadingDegrees=roundHeading)        rightDriverBumper.whileTrue(setPipeline0.andThen(approachTagRight))        # Operator Controller        leftOperatorBumper = self.operatorController.button(XboxController.Button.kLeftBumper)        leftOperatorBumper.onTrue(InstantCommand(self.elevator.switchUp, self.elevator))        rightOperatorBumper = self.operatorController.button(XboxController.Button.kRightBumper)        rightOperatorBumper.onTrue(InstantCommand(self.elevator.switchDown, self.elevator))        yOperatorButton = self.operatorController.button(XboxController.Button.kY)        singleIntakeCmd = IntakeGamepiece(self.singleIntake, speed=-0.4)        yOperatorButton.whileTrue(singleIntakeCmd)        xOperatorButton = self.operatorController.button(XboxController.Button.kX)        oppSingleIntake = IntakeGamepiece(self.singleIntake, speed=0.4)        xOperatorButton.whileTrue(oppSingleIntake)        aOperatorButton = self.operatorController.button(XboxController.Button.kA)        intakeCmd = IntakeGamepiece(self.intake, speed=-0.2)        aOperatorButton.whileTrue(intakeCmd)    def disablePIDSubsystems(self) -> None:        """Disables all ProfiledPIDSubsystem and PIDSubsystem instances.        This should be called on robot disable to prevent integral windup."""    def getAutonomousCommand(self) -> commands2.Command:        """        :returns: the command to run in autonomous        """        command = self.chosenAuto.getSelected()        return command()    def configureAutos(self):        self.chosenAuto = wpilib.SendableChooser()        # you can also set the default option, if needed        self.chosenAuto.addOption("Blue 1 Station", self.getBlue1Station)        self.chosenAuto.addOption("Blue 3 Station", self.getBlue3Station)        self.chosenAuto.addOption("Red 1 Station", self.getRed1Station)        self.chosenAuto.addOption("Red 3 Station", self.getRed3Station)        self.chosenAuto.addOption("Blue 1", self.getBlue1)        self.chosenAuto.addOption("Blue 2", self.getBlue2)        self.chosenAuto.addOption("Blue 3", self.getBlue3)        self.chosenAuto.addOption("Red 1", self.getRed1)        self.chosenAuto.addOption("Red 2", self.getRed2)        self.chosenAuto.addOption("Red 3", self.getRed3)        wpilib.SmartDashboard.putData("Chosen Auto", self.chosenAuto)    def getBlue1Station(self):        #Start to Reef        startLocation = ResetXY(x=7.600, y=6.150, headingDegrees=180, drivetrain=self.robotDrive)        setPipeline3 = SetCameraPipeline(camera=self.limelight, pipelineIndex=3)        aimToTwoForty = AimToDirection(degrees=-120, drivetrain=self.robotDrive)        approachTag = ApproachTag(camera=self.limelight, drivetrain=self.robotDrive, speed=1.0, pushForwardSeconds=None,                                      specificHeadingDegrees=-120)        score = IntakeFeedGamepieceForward(self.intake, motor1speed=-0.5, motor2speed=-0.2).withTimeout(1.5)        startToReef = startLocation.andThen(setPipeline3).andThen(aimToTwoForty).andThen(approachTag).andThen(score)        #Reef to Station        swerveToFrontStation = SwerveToPoint(x=4.120, y=6.150, headingDegrees=-135, drivetrain=self.robotDrive, speed=0.5)        aimToOneTwentyFive = AimToDirection(degrees=125, drivetrain=self.robotDrive)        approachTagStation = ApproachTag(camera=self.limelight, drivetrain=self.robotDrive, speed=1.0, pushForwardSeconds=None,                                         specificHeadingDegrees=-55)        intakeCmd = IntakeGamepiece(self.intake, speed=-0.4)        intakeCmd.isFinished = lambda: self.intake.isGamepieceInside()        reefToStation = swerveToFrontStation.andThen(aimToOneTwentyFive).andThen(approachTagStation).andThen(intakeCmd)        #Station to Feed        setPipeline3 = SetCameraPipeline(camera=self.limelight, pipelineIndex=3)        approachTagFeed = ApproachTag(camera=self.limelight, drivetrain=self.robotDrive, speed=1.0, pushForwardSeconds=None,                                            specificHeadingDegrees=-135)        lowFeed = IntakeFeedGamepieceForward(self.intake, motor1speed=-0.5, motor2speed=-0.2).withTimeout(1.5)        currentHeading = lambda: self.robotDrive.getHeading().degrees()        moveBack = SwerveMove(metersToTheLeft=0.0, metersBackwards=1, drivetrain=self.robotDrive, speed=0.5)        littleDance = AimToDirection(degrees=lambda: currentHeading() + 180, drivetrain=self.robotDrive, speed=0.2)        stationToFeed = setPipeline3.andThen(approachTagFeed).andThen(lowFeed).andThen(moveBack).andThen(littleDance)        command = startToReef.andThen(reefToStation).andThen(stationToFeed)        return command    def getBlue3Station(self):        #Start to Reef        startLocation = ResetXY(x=7.600, y=1.847, headingDegrees=180, drivetrain=self.robotDrive)        setPipeline3 = SetCameraPipeline(camera=self.limelight, pipelineIndex=3)        aimToOneTwenty = AimToDirection(degrees=120, drivetrain=self.robotDrive)        approachTag = ApproachTag(camera=self.limelight, drivetrain=self.robotDrive, speed=1.0, pushForwardSeconds=None,                                      specificHeadingDegrees=120)        score = IntakeFeedGamepieceForward(self.intake, motor1speed=-0.5, motor2speed=-0.2).withTimeout(1.5)        startToReef = startLocation.andThen(setPipeline3).andThen(aimToOneTwenty).andThen(approachTag).andThen(score)        #Reef to Station        swerveToFrontStation = SwerveToPoint(x=4.306, y=1.847, headingDegrees=-150, drivetrain=self.robotDrive, speed=0.5)        approachTagStation = ApproachTag(camera=self.limelight, drivetrain=self.robotDrive, speed=1.0, pushForwardSeconds=None,                                         specificHeadingDegrees=-125)        intakeCmd = IntakeGamepiece(self.intake, speed=-0.4)        intakeCmd.isFinished = lambda: self.intake.isGamepieceInside()        reefToStation = swerveToFrontStation.andThen(approachTagStation).andThen(intakeCmd)        #Station to Feed        setPipeline3 = SetCameraPipeline(camera=self.limelight, pipelineIndex=3)        approachTagFeed = ApproachTag(camera=self.limelight, drivetrain=self.robotDrive, speed=1.0, pushForwardSeconds=None,                                            specificHeadingDegrees=60)        lowFeed = IntakeFeedGamepieceForward(self.intake, motor1speed=-0.5, motor2speed=-0.2).withTimeout(1.5)        currentHeading = lambda: self.robotDrive.getHeading().degrees()        moveBack = SwerveMove(metersToTheLeft=0.0, metersBackwards=1, drivetrain=self.robotDrive, speed=0.5)        littleDance = AimToDirection(degrees=lambda: currentHeading() + 180, drivetrain=self.robotDrive, speed=0.2)        stationToFeed = setPipeline3.andThen(approachTagFeed).andThen(lowFeed).andThen(moveBack).andThen(littleDance)        command = startToReef.andThen(reefToStation).andThen(stationToFeed)        return command    def getRed1Station(self):        #Start to Reef        startLocation = ResetXY(x=10.000, y=6.150, headingDegrees=0, drivetrain=self.robotDrive)        setPipeline3 = SetCameraPipeline(camera=self.limelight, pipelineIndex=3)        aimToOneTwenty = AimToDirection(degrees=-60, drivetrain=self.robotDrive)        approachTag = ApproachTag(camera=self.limelight, drivetrain=self.robotDrive, speed=1.0, pushForwardSeconds=None,                                      specificHeadingDegrees=-60)        score = IntakeFeedGamepieceForward(self.intake, motor1speed=-0.5, motor2speed=-0.2).withTimeout(1.5)        startToReef = startLocation.andThen(setPipeline3).andThen(aimToOneTwenty).andThen(approachTag).andThen(score)        #Reef to Station        swerveToFrontStation = SwerveToPoint(x=13.068, y=6.285, headingDegrees=25, drivetrain=self.robotDrive, speed=0.5)        setPipeline1 = SetCameraPipeline(camera=self.limelight, pipelineIndex=1)        approachTagStation = ApproachTag(camera=self.limelight, drivetrain=self.robotDrive, speed=1.0, pushForwardSeconds=None,                                         specificHeadingDegrees=-125)        intakeCmd = IntakeGamepiece(self.intake, speed=-0.4)        intakeCmd.isFinished = lambda: self.intake.isGamepieceInside()        reefToStation = swerveToFrontStation.andThen(setPipeline1).andThen(approachTagStation).andThen(intakeCmd)        #Station to Feed        setPipeline3 = SetCameraPipeline(camera=self.limelight, pipelineIndex=3)        approachTagFeed = ApproachTag(camera=self.limelight, drivetrain=self.robotDrive, speed=1.0, pushForwardSeconds=None,                                            specificHeadingDegrees=-120)        lowFeed = IntakeFeedGamepieceForward(self.intake, motor1speed=-0.5, motor2speed=-0.2).withTimeout(1.5)        currentHeading = lambda: self.robotDrive.getHeading().degrees()        moveBack = SwerveMove(metersToTheLeft=0.0, metersBackwards=1, drivetrain=self.robotDrive, speed=0.5)        littleDance = AimToDirection(degrees=lambda: currentHeading() + 180, drivetrain=self.robotDrive, speed=0.2)        stationToFeed = setPipeline3.andThen(approachTagFeed).andThen(lowFeed).andThen(moveBack).andThen(littleDance)        command = startToReef.andThen(reefToStation).andThen(stationToFeed)        return command    def getRed3Station(self):        #Start to Reef        startLocation = ResetXY(x=10.000, y=1.847, headingDegrees=0, drivetrain=self.robotDrive)        setPipeline3 = SetCameraPipeline(camera=self.limelight, pipelineIndex=3)        aimToSixty = AimToDirection(degrees=60, drivetrain=self.robotDrive)        approachTag = ApproachTag(camera=self.limelight, drivetrain=self.robotDrive, speed=1.0, pushForwardSeconds=None,                                      specificHeadingDegrees=60)        score = IntakeFeedGamepieceForward(self.intake, motor1speed=-0.5, motor2speed=-0.2).withTimeout(1.5)        startToReef = startLocation.andThen(setPipeline3).andThen(aimToSixty).andThen(approachTag).andThen(score)        #Reef to Station        swerveToFrontStation = SwerveToPoint(x=12.975, y=1.672, headingDegrees=-30, drivetrain=self.robotDrive, speed=0.5)        setPipeline1 = SetCameraPipeline(camera=self.limelight, pipelineIndex=1)        approachTagStation = ApproachTag(camera=self.limelight, drivetrain=self.robotDrive, speed=1.0, pushForwardSeconds=None,                                         specificHeadingDegrees=125)        intakeCmd = IntakeGamepiece(self.intake, speed=-0.4)        intakeCmd.isFinished = lambda: self.intake.isGamepieceInside()        reefToStation = swerveToFrontStation.andThen(setPipeline1).andThen(approachTagStation).andThen(intakeCmd)        #Station to Feed        setPipeline3 = SetCameraPipeline(camera=self.limelight, pipelineIndex=3)        approachTagFeed = ApproachTag(camera=self.limelight, drivetrain=self.robotDrive, speed=1.0, pushForwardSeconds=None,                                            specificHeadingDegrees=120)        lowFeed = IntakeFeedGamepieceForward(self.intake, motor1speed=-0.5, motor2speed=-0.2).withTimeout(1.5)        currentHeading = lambda: self.robotDrive.getHeading().degrees()        moveBack = SwerveMove(metersToTheLeft=0.0, metersBackwards=1, drivetrain=self.robotDrive, speed=0.5)        littleDance = AimToDirection(degrees=lambda: currentHeading() + 180, drivetrain=self.robotDrive, speed=0.2)        stationToFeed = setPipeline3.andThen(approachTagFeed).andThen(lowFeed).andThen(moveBack).andThen(littleDance)        command = startToReef.andThen(reefToStation).andThen(stationToFeed)        return command    def getBlue1(self):        startLocation = ResetXY(x=7.600, y=6.150, headingDegrees=180, drivetrain=self.robotDrive)        setPipeline3 = SetCameraPipeline(camera=self.limelight, pipelineIndex=3)        aimToTwoForty = AimToDirection(degrees=240, drivetrain=self.robotDrive)        approachTag = ApproachTag(camera=self.limelight, drivetrain=self.robotDrive, speed=1.0, pushForwardSeconds=None,                                  specificHeadingDegrees=240)        score = IntakeFeedGamepieceForward(self.intake, motor1speed=-0.5, motor2speed=-0.2).withTimeout(1.5)        command = startLocation.andThen(setPipeline3).andThen(aimToTwoForty).andThen(approachTag).andThen(score)        return command    def getBlue2(self):        startLocation = ResetXY(x=7.600, y=4.000, headingDegrees=180, drivetrain=self.robotDrive)        setPipeline3 = SetCameraPipeline(camera=self.limelight, pipelineIndex=3)        approachTag = ApproachTag(camera=self.limelight, drivetrain=self.robotDrive, speed=1.0, pushForwardSeconds=None,                                  specificHeadingDegrees=180)        score = IntakeFeedGamepieceForward(self.intake, motor1speed=-0.5, motor2speed=-0.2).withTimeout(1.5)        command = startLocation.andThen(setPipeline3).andThen(approachTag).andThen(score)        return command    def getBlue3(self):        startLocation = ResetXY(x=7.600, y=1.847, headingDegrees=180, drivetrain=self.robotDrive)        setPipeline2 = SetCameraPipeline(camera=self.limelight, pipelineIndex=2)        aimToOneTwenty = AimToDirection(degrees=120, drivetrain=self.robotDrive)        approachTag = ApproachTag(camera=self.limelight, drivetrain=self.robotDrive, speed=1.0, pushForwardSeconds=None,                                  specificHeadingDegrees=120)        score = IntakeFeedGamepieceForward(self.intake, motor1speed=-0.5, motor2speed=-0.2).withTimeout(1.5)        command = startLocation.andThen(setPipeline2).andThen(aimToOneTwenty).andThen(approachTag).andThen(score)        return command    def getRed1(self):        startLocation = ResetXY(x=10.000, y=6.150, headingDegrees=0, drivetrain=self.robotDrive)        setPipeline3 = SetCameraPipeline(camera=self.limelight, pipelineIndex=3)        aimToOneTwenty = AimToDirection(degrees=-60, drivetrain=self.robotDrive)        approachTag = ApproachTag(camera=self.limelight, drivetrain=self.robotDrive, speed=1.0, pushForwardSeconds=None,                                  specificHeadingDegrees=-60)        score = IntakeFeedGamepieceForward(self.intake, motor1speed=-0.5, motor2speed=-0.2).withTimeout(1.5)        command = startLocation.andThen(setPipeline3).andThen(aimToOneTwenty).andThen(approachTag).andThen(score)        return command    def getRed2(self):        startLocation = ResetXY(x=10.000, y=4.000, headingDegrees=0, drivetrain=self.robotDrive)        setPipeline2 = SetCameraPipeline(camera=self.limelight, pipelineIndex=2)        approachTag = ApproachTag(camera=self.limelight, drivetrain=self.robotDrive, speed=1.0, pushForwardSeconds=None,                                  specificHeadingDegrees=0)        score = IntakeFeedGamepieceForward(self.intake, motor1speed=-0.5, motor2speed=-0.2).withTimeout(1.5)        command = startLocation.andThen(setPipeline2).andThen(approachTag).andThen(score)        return command    def getRed3(self):        startLocation = ResetXY(x=10.000, y=1.847, headingDegrees=0, drivetrain=self.robotDrive)        setPipeline3 = SetCameraPipeline(camera=self.limelight, pipelineIndex=3)        aimToSixty = AimToDirection(degrees=60, drivetrain=self.robotDrive)        approachTag = ApproachTag(camera=self.limelight, drivetrain=self.robotDrive, speed=1.0, pushForwardSeconds=None,                                  specificHeadingDegrees=60)        score = IntakeFeedGamepieceForward(self.intake, motor1speed=-0.5, motor2speed=-0.2).withTimeout(1.5)        command = startLocation.andThen(setPipeline3).andThen(aimToSixty).andThen(approachTag).andThen(score)        return command    def getAutonomousTrajectoryExample(self) -> commands2.Command:        # Create config for trajectory        config = TrajectoryConfig(            AutoConstants.kMaxSpeedMetersPerSecond,            AutoConstants.kMaxAccelerationMetersPerSecondSquared,        )        # Add kinematics to ensure max speed is actually obeyed        config.setKinematics(DriveConstants.kDriveKinematics)        # An example trajectory to follow. All units in meters.        exampleTrajectory = TrajectoryGenerator.generateTrajectory(            # Start at the origin facing the +X direction            Pose2d(0, 0, Rotation2d(0)),            # Pass through these two interior waypoints, making an 's' curve path            [Translation2d(0.5, 0.5), Translation2d(1, -0.5)],            # End 1.5 meters straight ahead of where we started, facing forward            Pose2d(1.5, 0, Rotation2d(0)),            config,        )        thetaController = ProfiledPIDControllerRadians(            AutoConstants.kPThetaController,            0,            0,            AutoConstants.kThetaControllerConstraints,        )        thetaController.enableContinuousInput(-math.pi, math.pi)        driveController = HolonomicDriveController(            PIDController(AutoConstants.kPXController, 0, 0),            PIDController(AutoConstants.kPXController, 0, 0),            thetaController,        )        swerveControllerCommand = commands2.SwerveControllerCommand(            exampleTrajectory,            self.robotDrive.getPose,  # Functional interface to feed supplier            DriveConstants.kDriveKinematics,            driveController,            self.robotDrive.setModuleStates,            (self.robotDrive,),        )        # Reset odometry to the starting pose of the trajectory.        self.robotDrive.resetOdometry(exampleTrajectory.initialPose())        # Run path following command, then stop at the end.        return swerveControllerCommand.andThen(            cmd.run(                lambda: self.robotDrive.drive(0, 0, 0, False, False),                self.robotDrive,            )        )    def getTestCommand(self) -> typing.Optional[commands2.Command]:        """        :returns: the command to run in test mode (to exercise all systems)        """        return None