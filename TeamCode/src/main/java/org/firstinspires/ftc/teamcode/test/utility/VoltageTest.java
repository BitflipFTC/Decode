package org.firstinspires.ftc.teamcode.test.utility;

import com.acmerobotics.dashboard.FtcDashboard;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "Utility: Voltage kS Tuning", group = "Utility")
public class VoltageTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry, FtcDashboard.getInstance().getTelemetry());

        DcMotorEx fl = hardwareMap.get(DcMotorEx.class, "frontleft");
        DcMotorEx fr = hardwareMap.get(DcMotorEx.class, "frontright");
        DcMotorEx br = hardwareMap.get(DcMotorEx.class, "backright");
        DcMotorEx bl = hardwareMap.get(DcMotorEx.class, "backleft");
        waitForStart();
        double pow = 0;

        while(opModeIsActive()) {
            pow += ((gamepad1.a ? 1 : 0) - (gamepad1.b ? 1 : 0)) * 0.001;
            fl.setPower(pow);
            fr.setPower(pow);
            bl.setPower(pow);
            br.setPower(pow);
            telemetry.addData("Current",fl.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Power",pow);
            telemetry.update();
        }
    }
}
