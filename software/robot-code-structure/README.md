# Robot Code Structure

This year, we are using [pedropathing](https://pedropathing.com/docs/pathing) for the autonomous and also parts of the robot control. I would highly recommend reading through their documentation to get a handle on the coding examples/concepts that they have developed.

***

Below is the baseline code that would get you started on your journey.

## TeleOP

{% tabs %}
{% tab title="Vanilla" %}
This TeleOP uses the FTC SDK library, the simplest and closest you could get to raw code. This is pulled from the [gm0 mecanum teleop section](https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html).

### Robot Centric Final Code (make changes accordingly)

```java
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class MecanumTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
        }
    }
}
```
{% endtab %}

{% tab title="Pedro" %}
This TeleOP uses the [Pedro Pathing library](https://pedropathing.com/docs/pathing/guide/teleop-usage). This version has built-in localization and autonomous compatibility as long as your robot is tuned correctly ([Tuning Guide](https://pedropathing.com/docs/pathing/tuning)).

```java
package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "Example TeleOp")
public class ExampleTeleOp extends OpMode {

    private Follower follower;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
    }

    @Override
    public void loop() {
        follower.manual(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x
        );

        follower.update();
    }
}
```
{% endtab %}
{% endtabs %}

## Auto

Pedro Pathing offers an extremely useful and simple autonomous system that uses predetermined paths created via [Visualizer](https://visualizer.pedropathing.com/). To set up the code, [pedro's documentation](https://pedropathing.com/docs/pathing/guide/setting-up-auto) is the best at guiding you through the process. Legit couldn't explain it better.

