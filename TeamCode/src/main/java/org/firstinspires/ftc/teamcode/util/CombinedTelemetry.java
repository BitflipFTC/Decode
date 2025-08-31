package org.firstinspires.ftc.teamcode.util;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;


/* just instantiate a new CombinedTelemetry object and pass in the opmode\
 * it allows you to write telemetry to panels and the DS with one method
 * supports addData, addLine, and update methods
 */
public class CombinedTelemetry implements Telemetry {
    Telemetry dsTelemetry;
    TelemetryManager panelsTelemetry;

    public CombinedTelemetry (OpMode opMode) {
        dsTelemetry = opMode.telemetry;
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public Item addData(String caption, String format, Object... args) {
        return null;
    }

    @Override
    public Item addData(String caption, Object value) {
        dsTelemetry.addData(caption, value);
        panelsTelemetry.addData(caption, value);
        return null;
    }

    @Override
    public <T> Item addData(String caption, Func<T> valueProducer) {
        return null;
    }

    @Override
    public <T> Item addData(String caption, String format, Func<T> valueProducer) {
        return null;
    }

    @Override
    public boolean removeItem(Item item) {
        return false;
    }

    @Override
    public void clear() {

    }

    @Override
    public void clearAll() {

    }

    @Override
    public Object addAction(Runnable action) {
        return null;
    }

    @Override
    public boolean removeAction(Object token) {
        return false;
    }

    @Override
    public void speak(String text) {

    }

    @Override
    public void speak(String text, String languageCode, String countryCode) {

    }

    @Override
    public boolean update() {
        dsTelemetry.update();
        panelsTelemetry.update();
        return false;
    }

    @Override
    public Line addLine() {
        dsTelemetry.addLine();
        return null;
    }

    @Override
    public Line addLine(String lineCaption) {
        dsTelemetry.addLine(lineCaption);
        panelsTelemetry.addLine(lineCaption);
        return null;
    }

    @Override
    public boolean removeLine(Line line) {
        return false;
    }

    @Override
    public boolean isAutoClear() {
        return false;
    }

    @Override
    public void setAutoClear(boolean autoClear) {

    }

    @Override
    public int getMsTransmissionInterval() {
        return 0;
    }

    @Override
    public void setMsTransmissionInterval(int msTransmissionInterval) {

    }

    @Override
    public String getItemSeparator() {
        return "";
    }

    @Override
    public void setItemSeparator(String itemSeparator) {
        dsTelemetry.setItemSeparator(itemSeparator);
    }

    @Override
    public String getCaptionValueSeparator() {
        return "";
    }

    @Override
    public void setCaptionValueSeparator(String captionValueSeparator) {

    }

    @Override
    public void setDisplayFormat(DisplayFormat displayFormat) {

    }

    @Override
    public Log log() {
        return null;
    }
}
