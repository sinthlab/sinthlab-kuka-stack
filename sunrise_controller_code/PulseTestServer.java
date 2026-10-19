package lbr_fri_ros2;

import javax.inject.Inject;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;
import com.kuka.roboticsAPI.uiModel.IApplicationUI;

/**
 * PulseTestServer
 *
 * Tests the neural-sync TTL outputs (EL2124 outputs 1-4, see TtlPulser) WITHOUT moving the arm or
 * starting FRI. Run it from the SmartPad's application list like LbrImpedanceControlServer.
 *
 * It does two things at once until Cancel:
 *   1. SmartPad menu: pick an output (or "All"), a width and once / ten times, and it fires. "All"
 *      fires output N as N pulses (1 = one pulse, 2 = two, ...), so each BNC line identifies itself
 *      on the NSP or a scope.
 *   2. Remote commands on UDP TtlPulser.DEFAULT_PORT (30002), the same ones LbrImpedanceControlServer
 *      serves, so the network path from the ROS computer is tested too:
 *        echo "PULSE 1 10" | nc -u -w1 172.31.1.147 30002
 *
 * What to look for: the EL2124 channel LED blinks, a multimeter or scope on the BNC reads 0 V idle and
 * 5 V during the pulse, and the NSP analog channel shows the step. README, "Neural synchronisation".
 */
public class PulseTestServer extends RoboticsAPIApplication {
    @Inject
    private LBR lbr_;
    @Inject
    private IApplicationUI applicationUi;

    private TtlPulser ttl_;

    private static final String CANCEL = "Cancel";
    private static final String[] OUTPUTS = { "Output 1", "Output 2", "Output 3", "Output 4",
                                              "All (output N = N pulses)" };
    private static final String[] WIDTHS = { "10 ms", "50 ms", "100 ms", "500 ms" };
    private static final int[] WIDTH_MS = { 10, 50, 100, 500 };
    private static final String[] REPEATS = { "Once", "10 times, 1 per second" };
    private static final long GAP_MS = 250;      // between the pulses of one output in "All"

    private int ask(String question, String[] options) {
        String[] withCancel = new String[options.length + 1];
        System.arraycopy(options, 0, withCancel, 0, options.length);
        withCancel[options.length] = CANCEL;
        int i = applicationUi.displayModalDialog(ApplicationDialogType.QUESTION, question, withCancel);
        return i == options.length ? -1 : i;
    }

    @Override
    public void initialize() {
        ttl_ = TtlPulser.create(lbr_.getController());
    }

    @Override
    public void run() {
        if (!ttl_.isAvailable()) {
            getLogger().error("TTL outputs unavailable: " + ttl_.reason());
            applicationUi.displayModalDialog(ApplicationDialogType.ERROR,
                                             "TTL outputs unavailable:\n" + ttl_.reason(), "OK");
            return;
        }
        if (ttl_.listen(TtlPulser.DEFAULT_PORT)) {
            getLogger().info("Listening for pulse commands on UDP " + TtlPulser.DEFAULT_PORT
                             + " (PULSE <output> <width ms> | ALLOFF | STATUS).");
        } else {
            getLogger().warn("UDP " + TtlPulser.DEFAULT_PORT + " is busy (is LbrImpedanceControlServer running?);"
                             + " the SmartPad menu still works.");
        }
        while (true) {
            int out = ask("Pulse which output?", OUTPUTS);
            if (out < 0) return;
            int w = ask("Pulse width?", WIDTHS);
            if (w < 0) return;
            int rep = ask("How many times?", REPEATS);
            if (rep < 0) return;
            int times = rep == 0 ? 1 : 10;
            for (int n = 0; n < times; n++) {
                long t0 = System.currentTimeMillis();
                if (out < 4) {
                    ttl_.pulse(out + 1, WIDTH_MS[w]);
                } else {
                    fireAll(WIDTH_MS[w]);
                }
                if (n < times - 1) {
                    sleepMs(Math.max(0, 1000 - (System.currentTimeMillis() - t0)));
                }
            }
            getLogger().info("Sent " + OUTPUTS[out] + ", " + WIDTHS[w] + ", " + REPEATS[rep] + ".");
        }
    }

    /** Output N as N pulses, outputs one after another. */
    private void fireAll(int widthMs) {
        for (int ch = 1; ch <= TtlPulser.CHANNELS; ch++) {
            for (int k = 0; k < ch; k++) {
                ttl_.pulse(ch, widthMs);
                sleepMs(widthMs + GAP_MS);
            }
            sleepMs(2 * GAP_MS);
        }
    }

    private static void sleepMs(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    @Override
    public void dispose() {
        if (ttl_ != null) {
            ttl_.close();
        }
        super.dispose();
    }
}
