package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.OTOSConfig;
import com.pedropathing.revhub.localizers.OTOSLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static Follower createAutonomous(HardwareMap h) {
        return new Follower(
                new OTOSLocalizer(h,localizerConfig),
                new Mecanum(h, autonomousConfig),
                new Foresight(foresightConfig)
        );
    }

    public static Follower createTeleop(HardwareMap h) {
        return new Follower(
                new OTOSLocalizer(h,localizerConfig),
                new Mecanum(h, teleOpConfig),
                new Foresight(foresightConfig)
        );
    }

    public static final boolean MOTORS_MIRRORED_LEFT_RIGHT = true;

    private static void configureMotors(MecanumConfig c) {
        if (MOTORS_MIRRORED_LEFT_RIGHT) {
            c.frontLeftName.set("front_right");
            c.frontRightName.set("front_left");
            c.backLeftName.set("back_right");
            c.backRightName.set("back_left");
            c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
            c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
            c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
            c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
        } else {
            c.frontLeftName.set("front_left");
            c.frontRightName.set("front_right");
            c.backLeftName.set("back_left");
            c.backRightName.set("back_right");
            c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
            c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
            c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
            c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
        }
    }

    public static MecanumConfig autonomousConfig = new MecanumConfig(c -> {
        configureMotors(c);
        c.manualBrakeMode.set(false);
    });

    public static MecanumConfig teleOpConfig = new MecanumConfig(c -> {
        configureMotors(c);
        c.manualBrakeMode.set(true);
    });

    public static OTOSConfig localizerConfig = new OTOSConfig(c -> {
        c.name.set("otos");
        c.linearScalar.set(.954223883894058);
        c.angularScalar.set(0.9230812569545185);
        c.offset.set(new Pose(-0.9329296833884716, -.04700349987676078, 1.5894277110401616));
        c.linearUnit.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.16168643293822813);
                Controller secondaryTranslationalForward = Controller.proportional(0.05973881662961832);
                Controller primaryTranslationalLateral = Controller.proportional(0.19935880451707633);
                Controller secondaryTranslationalLateral = Controller.proportional(0.07365775130369488);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.017985081590509024));
                c.brake.set(Controller.proportionalFeedforward(0.01528731935193267));

                c.headingFeedback.set(Controller.proportional(2.4229562078195808));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.14334301631898125, -0.017861477852749146));

                c.linearBrakeCoefficients.set(Matrix.diag(0.08750010880235455, 0.05708137552628358));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.001398144515609741, 0.0019425346179843407));

                c.maxAchievableForwardVelocity.set(59.46403254309469);
                c.maxAchievableStrafeVelocity.set(45.6190347731692);
                c.naturalForwardDeceleration.set(41.667603183754466);
                c.naturalStrafeDeceleration.set(72.2324425860493);
            }
    );
}
