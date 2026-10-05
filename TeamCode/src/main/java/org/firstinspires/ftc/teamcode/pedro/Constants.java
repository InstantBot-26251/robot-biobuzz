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

    public static MecanumConfig autonomousConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("front_left");
        c.frontRightName.set("front_right");
        c.backLeftName.set("back_left");
        c.backRightName.set("back_right");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.manualBrakeMode.set(false);
    });

    public static MecanumConfig teleOpConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("front_left");
        c.frontRightName.set("front_right");
        c.backLeftName.set("back_left");
        c.backRightName.set("back_right");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.manualBrakeMode.set(true);
    });



    public static OTOSConfig localizerConfig = new OTOSConfig(c -> {
        c.name.set("otos");
        c.linearScalar.set(0.987169399555226);
        c.angularScalar.set(0.9797871068054063);
        c.offset.set(new Pose(-0.0, -0.0, 90));
        c.linearUnit.set(DistanceUnit.INCH);
    });

    // TODO: actually tune foresight
    // https://pedropathing.com/docs/pathing/tuning/foresight
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.16147263595781208);
                Controller secondaryTranslationalForward = Controller.proportional(0.05965982435811513);
                Controller primaryTranslationalLateral = Controller.proportional(0.18042489639540243);
                Controller secondaryTranslationalLateral = Controller.proportional(0.06666217817607911);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.02040485318466872));
                c.brake.set(Controller.proportionalFeedforward(0.01734412520696841));

                c.headingFeedback.set(Controller.proportional(2.2819753429709464));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.22998913757051062, -0.04678451731386972));

                c.linearBrakeCoefficients.set(Matrix.diag(0.03278634870894801, 0.07084676408875436));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.002776367236773565, 0.0016859571844903055));

                c.maxAchievableForwardVelocity.set(56.83166983299488);
                c.maxAchievableStrafeVelocity.set(45.97460074823986);
                c.naturalForwardDeceleration.set(49.350365188768514);
                c.naturalStrafeDeceleration.set(65.13648242339649);
                        }
                );
}