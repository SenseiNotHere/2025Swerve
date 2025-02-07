
from commands2 import Subsystem
from rev import SparkMax, SparkBase, SparkLowLevel, SparkBaseConfig, LimitSwitchConfig
from wpilib import SmartDashboard


class Intake(Subsystem):
    def __init__(self, leaderCanID, leaderInverted=True, followerCanID=None, followerInverted=False) -> None:
        super().__init__()

        # 1. setup the leader motor
        self.motor = SparkMax(leaderCanID, SparkLowLevel.MotorType.kBrushless)

        self.motorConfig = SparkBaseConfig()
        self.motorConfig.inverted(leaderInverted)
        self.motorConfig.setIdleMode(SparkBaseConfig.IdleMode.kBrake)
        self.motorConfig.limitSwitch.forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyClosed)
        self.motorConfig.limitSwitch.forwardLimitSwitchEnabled(True)
        self.motor.configure(self.motorConfig,
                             SparkBase.ResetMode.kResetSafeParameters,
                             SparkBase.PersistMode.kPersistParameters)

        # when the gamepiece is fully in, it will touch the limit switch -- physical or optical
        self.limitSwitch = self.motor.getForwardLimitSwitch()

        # 2. setup the follower motor, if followerCanID is not None
        self.followerMotor = None
        self.followerConfig = None
        if followerCanID is not None:
            self.followerConfig = SparkBaseConfig()
            self.followerConfig.follow(leaderCanID, leaderInverted != followerInverted)
            self.notFollowingConfig = SparkBaseConfig()
            self.notFollowingConfig.inverted(followerInverted)
            self.notFollowingConfig.setIdleMode(SparkBaseConfig.IdleMode.kBrake)
            self.notFollowingConfig.limitSwitch.forwardLimitSwitchEnabled(False)
            self.followerMotor = SparkMax(followerCanID, SparkLowLevel.MotorType.kBrushless)
            self.followerMotor.configure(self.followerConfig,
                                         SparkBase.ResetMode.kResetSafeParameters,
                                         SparkBase.PersistMode.kPersistParameters)
        self.following = self.followerMotor is not None

        # 3. safe initial state
        self._setSpeed(0.0)


    def enableLimitSwitch(self):
        self.motorConfig.limitSwitch.forwardLimitSwitchEnabled(True)
        self.motor.configure(self.motorConfig,
            SparkBase.ResetMode.kNoResetSafeParameters,
            SparkBase.PersistMode.kNoPersistParameters)
        if self.followerMotor is not None:
            self.followerMotor.configure(
                self.followerConfig,
                SparkBase.ResetMode.kNoResetSafeParameters,
                SparkBase.PersistMode.kNoPersistParameters)
            self.following = True
        # ^^ do not reset and do not persist, just enable the switch

    def disableLimitSwitch(self):
        self.motorConfig.limitSwitch.forwardLimitSwitchEnabled(False)
        self.motor.configure(self.motorConfig,
            SparkBase.ResetMode.kNoResetSafeParameters,
            SparkBase.PersistMode.kNoPersistParameters)
        if self.followerMotor is not None:
            self.followerMotor.configure(
                self.notFollowingConfig,
                SparkBase.ResetMode.kNoResetSafeParameters,
                SparkBase.PersistMode.kNoPersistParameters)
            self.following = False
        # ^^ do not reset and do not persist, just disable the switch

    def isGamepieceInside(self) -> bool:
        return self.limitSwitch.get()

    def noGamepieceInside(self) -> bool:
        return not self.isGamepieceInside()

    def periodic(self):
        SmartDashboard.putBoolean("intakeFull", self.limitSwitch.get())

    def intakeGamepiece(self, speed=0.25):
        """
        If the gamepiece is not inside, try to intake it
        """
        self.enableLimitSwitch()
        self._setSpeed(speed)
        print("Intake::intakeGamepiece")

    def feedGamepieceForward(self, speed=1.0, motor2speed=None):
        """
        Rush the gamepiece forward into the shooter, at full speed (100%)
        """
        self.disableLimitSwitch()
        self._setSpeed(speed, motor2speed)
        print("Intake::feedGamepieceForward")

    def ejectGamepieceBackward(self, speed=0.25, motor2speed=None):
        """
        Eject the gamepiece back out of the intake
        """
        self.disableLimitSwitch()
        self._setSpeed(-speed, -motor2speed if motor2speed is not None else -speed)
        print("Intake::ejectGamepiece")

    def intakeGamepieceDespiteLimitSwitch(self, speed=0.25):
        """
        Even if (possibly broken) limit switch thinks that the gamepiece is already inside, try to intake it
        """
        self.disableLimitSwitch()
        self._setSpeed(speed)

    def stop(self):
        self._setSpeed(0)
        print("Intake::stop")

    def _setSpeed(self, motor1speed, motor2speed=None):
        self.motor.set(motor1speed)
        if motor2speed is None or self.following:
            motor2speed = motor1speed
        if not self.following:
            self.followerMotor.set(motor2speed)
        SmartDashboard.putNumber("intakeSpeed", motor1speed)
        SmartDashboard.putNumber("intakeSpeed2", motor2speed)
