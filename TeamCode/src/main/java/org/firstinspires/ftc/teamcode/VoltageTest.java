package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "volt test")
public class VoltageTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        DcMotorEx fl = hardwareMap.get(DcMotorEx.class, "frontright");
        waitForStart();
        double pow = 0;

        while(opModeIsActive()) {
            pow += ((gamepad1.a ? 1 : 0) - (gamepad1.b ? 1 : 0)) * 0.001;
            fl.setPower(pow);
            telemetry.addData("Current",fl.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Power",pow);
            telemetry.update();
        }
    }
}
