package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.Commands;
import com.seattlesolvers.solverslib.command.Robot;
import com.seattlesolvers.solverslib.command.Subsystem;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.depollinator.Depollinator;
import org.firstinspires.ftc.teamcode.depollinator.DepollinatorCommands;
import org.firstinspires.ftc.teamcode.intake.Intake;
import org.firstinspires.ftc.teamcode.intake.IntakeCommands;
import org.firstinspires.ftc.teamcode.opmodes.auto.AutoConstants;
import org.firstinspires.ftc.teamcode.chassis.Chassis;
import org.firstinspires.ftc.teamcode.chassis.ChassisCommands;
import org.firstinspires.ftc.teamcode.chassis.DriveConstants;
import org.firstinspires.ftc.teamcode.shooter.Shooter;
import org.firstinspires.ftc.teamcode.shooter.ShooterCommands;
import org.firstinspires.ftc.teamcode.shooter.ShooterConstants;
import org.firstinspires.ftc.teamcode.util.PanelsDrawing;
import org.firstinspires.ftc.teamcode.util.SubsystemIF;
import org.firstinspires.ftc.teamcode.vision.Limelight;
import org.firstinspires.ftc.teamcode.vision.VisionConstants;
import java.util.ArrayList;
import java.util.List;

public class RobotContainer extends Robot {
    public static Pose ROBOT_POSE = null;
    public static boolean POSE_FROM_AUTO = false;
    private final List<SubsystemIF> subsystems = new ArrayList<>();
    private final ElapsedTime timer = new ElapsedTime();
    private final List<LynxModule> hubs;
    private final Chassis chassis;
    private final Limelight limelight;
    private final Shooter shooter;
    private final Intake intake;

    private final Depollinator depollinator;

    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private GamepadEx gamepad1;
    private GamepadEx gamepad2;
    private final Gamepad rawGamepad1;
    private final Gamepad rawGamepad2;
    private boolean teleop = false;
    private boolean wasShooterReady = false;
    private boolean wasAimLocked = false;
    private final ElapsedTime aimSettledTimer = new ElapsedTime();
    private boolean secondGroup = false;
    private final ElapsedTime shooterSteadyTimer = new ElapsedTime();

    public RobotContainer(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) {
        reset();

        this.hardwareMap = hardwareMap;
        this.telemetry = new JoinedTelemetry(
                telemetry,
                PanelsTelemetry.INSTANCE.getFtcTelemetry()
        );
        this.gamepad1 = new GamepadEx(gamepad1);
        this.gamepad2 = new GamepadEx(gamepad2);
        this.rawGamepad1 = gamepad1;
        this.rawGamepad2 = gamepad2;
        PanelsDrawing.init();

        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        chassis = new Chassis(this.telemetry, hardwareMap);
        limelight = new Limelight(this.telemetry, hardwareMap, chassis);
        shooter = new Shooter(this.telemetry, hardwareMap);
        intake = new Intake(this.telemetry, hardwareMap);
        depollinator = new Depollinator(this.telemetry, hardwareMap);

        shooter.setDistanceSupplier(() -> limelight.getDistance(chassis.getPose()));

        subsystems.addAll(List.of(chassis, limelight, intake, shooter, depollinator));

        for(Subsystem s : subsystems) {
            register(s);
        }
    }

    @Override
    public void reset() {
        CommandScheduler.getInstance().reset();
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().clearButtons();
    }

    public void autonomousInit() {
        for(SubsystemIF s : subsystems) {
            s.autonomousInit();
        }

        // In auto, never fire off target or under speed: a held ball beats a missed one
        shooter.setForcedFeedAllowed(false);
        shooter.setFixedAutoRpm(() -> AutoConstants.AUTO_SHOT_RPM);
        shooter.setExtraRpm(() -> AutoConstants.AUTO_RPM_OFFSET
                + (secondGroup ? AutoConstants.SECOND_GROUP_EXTRA_RPM : 0));
        shooter.setFeedPermission(() -> !AutoConstants.AUTO_AIM || aimSettled());
    }

    public void teleopInit() {
        for(SubsystemIF s : subsystems) {
            s.teleopInit();
        }

        teleop = true;

        chassis.setDefaultCommand(ChassisCommands.createTeleopDriveCommand(
                chassis,
                limelight,
                () -> gamepad1.getLeftY() * driveScale(),
                () -> -gamepad1.getLeftX() * driveScale(),
                () -> -gamepad1.getRightX() * turnScale(),
                this::aimHeld));

        shooter.setFeedPermission(() -> !aimHeld() || aimLocked());

        gamepad1.getGamepadButton(GamepadKeys.Button.OPTIONS)
                        .whenPressed(chassis::resetHeading);

        gamepad2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(ShooterCommands.enableShooter(shooter))
                .whenReleased(ShooterCommands.disableShooter(shooter));
        new Trigger(() -> gamepad2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.2)
                .whenActive(ShooterCommands.shoot(shooter))
                .whenInactive(ShooterCommands.releaseShoot(shooter,
                        () -> gamepad2.getButton(GamepadKeys.Button.RIGHT_BUMPER)));
        new Trigger(() -> gamepad2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.2)
                .whenActive(ShooterCommands.holdFixedRpm(shooter))
                .whenInactive(ShooterCommands.releaseFixedRpm(shooter));

        gamepad2.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(ShooterCommands.trimUp(shooter));
        gamepad2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(ShooterCommands.trimDown(shooter));
        gamepad2.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(ShooterCommands.resetTrim(shooter));
        gamepad2.getGamepadButton(GamepadKeys.Button.A)
                .whenActive(IntakeCommands.intake(intake))
                .whenInactive(IntakeCommands.stopIntake(intake));
        gamepad1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(DepollinatorCommands.depollinate(depollinator))
                .whenReleased(DepollinatorCommands.restDepollinator(depollinator));
        gamepad2.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(IntakeCommands.outtake(intake))
                .whenReleased(IntakeCommands.stopIntake(intake));
    }

    private boolean aimHeld() {
        return gamepad1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.3;
    }

    private boolean slowHeld() {
        return gamepad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.3;
    }

    private double driveScale() {
        return slowHeld() ? DriveConstants.SLOW_MODE_DRIVE_SCALE : 1.0;
    }

    private double turnScale() {
        return slowHeld() ? DriveConstants.SLOW_MODE_TURN_SCALE : 1.0;
    }

    public boolean aimLocked() {
        double error = limelight.getAimErrorDeg(chassis.getPose());
        return !Double.isNaN(error) && Math.abs(error) <= DriveConstants.AIM_FEED_TOLERANCE_DEG;
    }

    private boolean aimSteady() {
        return aimLocked() && Math.abs(chassis.getAngularVelocity()) < Math.toRadians(AutoConstants.AIM_SETTLE_MAX_DEG_PER_SEC);
    }

    // On target and not turning for a moment, so the target isn't still based on frames from the spin
    private boolean aimSettled() {
        return aimSteady() && aimSettledTimer.milliseconds() >= AutoConstants.AIM_SETTLE_MS;
    }

    private boolean shooterOnSpeed() {
        return shooter.getTargetRpm() > 0
                && Math.abs(shooter.getTargetRpm() - shooter.getShooterRpm()) <= ShooterConstants.SHOOTER_VELOCITY_TOLERANCE;
    }

    // Shooter has held its target speed for a while, so it's done swinging after the last shot
    private boolean shooterSteady() {
        return shooterOnSpeed() && shooterSteadyTimer.milliseconds() >= AutoConstants.GROUP2_STEADY_MS;
    }

    private boolean readyToShoot() {
        return shooter.atTargetVelocity()
                && (shooter.isFixedRpmHeld() || limelight.getLiveTagCount() >= VisionConstants.READY_MIN_TAGS)
                && (!aimHeld() || aimLocked());
    }

    private void rumbleFeedback() {
        boolean ready = DriveConstants.RUMBLE_FEEDBACK && readyToShoot();
        boolean locked = aimHeld() && aimLocked();
        if (ready && !wasShooterReady) rawGamepad2.rumble(Gamepad.RUMBLE_DURATION_CONTINUOUS);
        if (!ready && wasShooterReady) rawGamepad2.stopRumble();
        if (DriveConstants.RUMBLE_FEEDBACK && locked && !wasAimLocked) rawGamepad1.rumble(150);
        wasShooterReady = ready;
        wasAimLocked = locked;
    }

    public void stop() {
        ROBOT_POSE = chassis.getPose();
        POSE_FROM_AUTO = !teleop;
        if (wasShooterReady) rawGamepad2.stopRumble();
        limelight.stop();
    }

    public void setPose(Pose pose) {
        chassis.setPose(pose);
    }

    public Command followPath(Path path) {
        return ChassisCommands.createFollowPathCommand(chassis, path);
    }

    public Command followPath(Path path, boolean holdEnd) {
        return ChassisCommands.createFollowPathCommand(chassis, path, holdEnd);
    }

    public Command enableShooter() {
        return ShooterCommands.enableShooter(shooter);
    }

    public Command aim() {
        return ChassisCommands.createTeleopDriveCommand(chassis, limelight, () -> 0, () -> 0, () -> 0, () -> true);
    }

    public Command aimAndShoot(long shootTime) {
        boolean useAim = AutoConstants.AUTO_AIM;
        Command shooting = Commands.sequence(
                Commands.race(
                        Commands.waitUntil(() -> shooter.atTargetVelocity() && (!useAim || aimSettled())),
                        Commands.waitMillis(ShooterConstants.SPIN_UP_TIME_MS)),
                ShooterCommands.shoot(shooter),
                Commands.waitMillis(shootTime),
                ShooterCommands.disableShooter(shooter));
        if (!useAim) {
            return Commands.sequence(
                    ShooterCommands.enableShooter(shooter),
                    Commands.race(
                            Commands.waitUntil(limelight::hasTarget),
                            Commands.waitMillis(AutoConstants.TARGET_WAIT_TIME)),
                    shooting);
        }
        // Feed, pause so the shooter recovers, then feed the rest
        Command timedShooting = Commands.sequence(
                Commands.runOnce(() -> secondGroup = false),
                Commands.race(
                        Commands.waitUntil(() -> shooter.atTargetVelocity() && aimSettled()),
                        Commands.waitMillis(ShooterConstants.SPIN_UP_TIME_MS)),
                Commands.runOnce(() -> shooter.setFeedOverride(true)),
                Commands.waitMillis(AutoConstants.FIRST_FEED_TIME_MS),
                // Group 2 gets its own speed; switching now lets the shooter reach it during the pause
                Commands.runOnce(() -> {
                    shooter.setFeedOverride(false);
                    secondGroup = true;
                }),
                Commands.waitMillis(AutoConstants.FEED_PAUSE_TIME_MS),
                // Don't start group 2 until the speed has settled after group 1
                Commands.race(
                        Commands.waitUntil(this::shooterSteady),
                        Commands.waitMillis(AutoConstants.GROUP2_MAX_WAIT_MS)),
                Commands.runOnce(() -> shooter.setFeedOverride(true)),
                Commands.waitMillis(AutoConstants.SECOND_FEED_TIME_MS),
                Commands.runOnce(() -> shooter.setFeedOverride(false)),
                // Keep the wheels at speed so the last balls still in the feed get shot
                Commands.waitMillis(AutoConstants.SPIN_AFTER_FEED_MS),
                ShooterCommands.disableShooter(shooter));
        // Spin until a tag is seen. If the time runs out, don't fire in a random direction
        return Commands.sequence(
                ShooterCommands.enableShooter(shooter),
                Commands.race(
                        Commands.waitUntil(limelight::isLive),
                        Commands.waitMillis(AutoConstants.TARGET_SEARCH_TIME),
                        spin(AutoConstants.TARGET_SEARCH_TURN_POWER)),
                new ConditionalCommand(
                        Commands.race(aim(), timedShooting),
                        ShooterCommands.disableShooter(shooter),
                        limelight::isLive));
    }

    private Command spin(double turnPower) {
        return ChassisCommands.createTeleopDriveCommand(chassis, limelight, () -> 0, () -> 0, () -> turnPower, () -> false);
    }

    public Command shootFor(long shootTime) {
        return aimAndShoot(shootTime);
    }

    public Chassis getChassis() {
        return chassis;
    }

    public Limelight getLimelight() {
        return limelight;
    }

    public Shooter getShooter() {
        return shooter;
    }

    public Telemetry getTelemetry() {
        return telemetry;
    }

    public void initLoop() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
        chassis.periodic();
        limelight.periodic();
        PanelsDrawing.update();
        telemetry.update();
    }

    public void periodic() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }

        if (!aimSteady()) aimSettledTimer.reset();
        if (!shooterOnSpeed()) shooterSteadyTimer.reset();
        run();
        if (teleop) rumbleFeedback();

        PanelsDrawing.update();
        telemetry.addLine();
        if (teleop) {
            telemetry.addData("Aim Locked", aimLocked());
            telemetry.addData("Slow Mode", slowHeld());
            telemetry.addData("Ready To Shoot", readyToShoot());
        }
        telemetry.addData("Loop Time", timer.milliseconds());
        telemetry.update();
        timer.reset();
    }
}
