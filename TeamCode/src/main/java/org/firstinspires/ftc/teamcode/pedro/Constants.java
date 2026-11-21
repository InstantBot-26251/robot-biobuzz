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
                new Mecanum(h, teleOpConfig),
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

    public static final boolean MOTORS_MIRRORED_LEFT_RIGHT = false;

    private static void configureMotors(MecanumConfig c) {
        if (MOTORS_MIRRORED_LEFT_RIGHT) {
            c.frontLeftName.set("front_right");
            c.frontRightName.set("front_left");
            c.backLeftName.set("back_right");
            c.backRightName.set("back_left");
            c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
            c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
            c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
            c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
        } else {
            c.frontLeftName.set("front_left");
            c.frontRightName.set("front_right");
            c.backLeftName.set("back_left");
            c.backRightName.set("back_right");

            c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
            c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
            c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
            c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
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
                Controller primaryTranslationalForward = Controller.proportional(0.19293966757048006);
                Controller secondaryTranslationalForward = Controller.proportional(0.07128605172442576);
                Controller primaryTranslationalLateral = Controller.proportional(0.27061138814383956);
                Controller secondaryTranslationalLateral = Controller.proportional(0.09998367704968471);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.01966502719393254));
                c.brake.set(Controller.proportionalFeedforward(0.016715273114842655));

                c.headingFeedback.set(Controller.proportional(2.755338091181044));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.08977640238002856, 0.0032815896489849193));

                c.linearBrakeCoefficients.set(Matrix.diag(0.11093975539888902, 0.0606173732508726));
                c.quadraticBrakeCoefficients.set(Matrix.diag(7.807505291924113E-4, 0.0020742858821903775));

                c.maxAchievableForwardVelocity.set(51.99622054651102);
                c.maxAchievableStrafeVelocity.set(41.3917346684332);
                c.naturalForwardDeceleration.set(51.648849407429886);
                c.naturalStrafeDeceleration.set(85.32181808227692);
            }
    );
}
