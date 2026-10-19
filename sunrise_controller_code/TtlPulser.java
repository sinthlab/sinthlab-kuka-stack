package lbr_fri_ros2;

import java.lang.reflect.Constructor;
import java.lang.reflect.Method;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;
import java.util.Locale;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;

/**
 * TtlPulser
 *
 * TTL pulses on the four 5 V outputs of the EL2124 in the Beckhoff stack on the cabinet's EtherCAT
 * (EK1100 -> EL9505 -> EL2124 -> EL9011), for neural sync with the Blackrock NSP. See the README,
 * "Neural synchronisation".
 *
 *   TtlPulser ttl = TtlPulser.create(lbr.getController());
 *   ttl.pulse(1, 10);          // output 1 high for 10 ms, then low; returns at once
 *
 * BINDING. The outputs are reached through the I/O group that Sunrise Workbench GENERATES from the
 * WorkVisual I/O configuration: group NspSync with BOOLEAN outputs Output1..Output4, mapped to the
 * EL2124 channels 1..4, gives the class com.kuka.generated.ioAccess.NspSyncIOGroup with setOutput1..4.
 * It is looked up by name at run time, so this file -- and every app using it -- compiles and runs
 * before that configuration exists: until then isAvailable() is false, reason() says why, and pulse()
 * does nothing but return false. If the group or outputs are named differently, change GROUP_CLASS /
 * OUTPUT_SETTER below, not the configuration.
 *
 * TIMING. The rising edge leaves when the cabinet writes the output, a few ms after pulse() is called,
 * with a few ms of jitter; the width is set by a Java timer, so it is good to a few ms. The edge on
 * the NSP is the time reference: it marks when the pulse was COMMANDED, so align events to it, and
 * use widths of 10 ms or more -- shorter ones can fall between I/O cycles.
 *
 * REMOTE COMMANDS (optional, listen()). One UDP datagram per command, ASCII, answered to the sender:
 *   PULSE <output 1-4> <width ms>   ->  OK PULSE 1 10      (or ERR <why>)
 *   ALLOFF                          ->  OK ALLOFF          (every output low)
 *   STATUS                          ->  OK available=true ... / ERR <reason>
 * e.g. from the ROS computer:  echo "PULSE 1 10" | nc -u -w1 172.31.1.147 30002
 */
public class TtlPulser {
    public static final String GROUP_CLASS = "com.kuka.generated.ioAccess.NspSyncIOGroup";
    public static final String OUTPUT_SETTER = "setOutput";      // + channel number: setOutput1 .. setOutput4
    public static final int CHANNELS = 4;
    public static final int MIN_WIDTH_MS = 1;
    public static final int MAX_WIDTH_MS = 5000;
    // UDP port for remote commands. The cabinet releases 30000-30010 to applications; 30001 is the
    // SmartPad settings report (cabinet -> ROS), 30200 is FRI.
    public static final int DEFAULT_PORT = 30002;

    private final Object group_;
    private final Method[] setters_ = new Method[CHANNELS + 1];   // index 1..4
    private final String reason_;
    private final ScheduledExecutorService io_;
    private final ScheduledFuture<?>[] off_ = new ScheduledFuture<?>[CHANNELS + 1];
    private volatile DatagramSocket socket_;

    /** Bind to the generated I/O group. Never throws: check isAvailable() / reason(). */
    public static TtlPulser create(Object controller) {
        try {
            Class<?> cls = Class.forName(GROUP_CLASS);
            Object group = null;
            for (Constructor<?> c : cls.getConstructors()) {
                Class<?>[] p = c.getParameterTypes();
                if (p.length == 1 && controller != null && p[0].isInstance(controller)) {
                    group = c.newInstance(controller);
                    break;
                }
            }
            if (group == null) {
                return new TtlPulser(null, null, GROUP_CLASS + " has no constructor taking the controller");
            }
            Method[] setters = new Method[CHANNELS + 1];
            for (int ch = 1; ch <= CHANNELS; ch++) {
                setters[ch] = cls.getMethod(OUTPUT_SETTER + ch, Boolean.class);
            }
            return new TtlPulser(group, setters, null);
        } catch (ClassNotFoundException e) {
            return new TtlPulser(null, null, "no I/O group " + GROUP_CLASS + ": the NspSync I/O configuration"
                                 + " is not in this project (README, Neural synchronisation)");
        } catch (NoSuchMethodException e) {
            return new TtlPulser(null, null, GROUP_CLASS + " lacks " + e.getMessage()
                                 + ": its outputs must be named Output1..Output4");
        } catch (Exception e) {
            return new TtlPulser(null, null, "could not open " + GROUP_CLASS + ": " + e);
        }
    }

    private TtlPulser(Object group, Method[] setters, String reason) {
        group_ = group;
        reason_ = reason;
        if (setters != null) {
            System.arraycopy(setters, 0, setters_, 0, setters.length);
        }
        // One thread does every output write, in order, so a caller never blocks on the I/O.
        io_ = Executors.newSingleThreadScheduledExecutor();
        if (group_ != null) {
            allLow();
        }
    }

    public boolean isAvailable() {
        return group_ != null;
    }

    /** Why the outputs are unavailable, or null when they are available. */
    public String reason() {
        return reason_;
    }

    /**
     * Output `channel` (1-4) high for `widthMs`, then low. Returns at once; false (and nothing sent) if
     * the outputs are unavailable or the arguments are out of range. A pulse on an output that is already
     * high extends it to end `widthMs` from now.
     */
    public synchronized boolean pulse(final int channel, int widthMs) {
        if (!isAvailable() || channel < 1 || channel > CHANNELS
            || widthMs < MIN_WIDTH_MS || widthMs > MAX_WIDTH_MS) {
            return false;
        }
        if (off_[channel] != null) {
            off_[channel].cancel(false);
        }
        io_.execute(new Runnable() {
            @Override
            public void run() {
                write(channel, true);
            }
        });
        off_[channel] = io_.schedule(new Runnable() {
            @Override
            public void run() {
                write(channel, false);
            }
        }, widthMs, TimeUnit.MILLISECONDS);
        return true;
    }

    /** Every output low now (cancels pending pulse ends). */
    public synchronized void allLow() {
        for (int ch = 1; ch <= CHANNELS; ch++) {
            if (off_[ch] != null) {
                off_[ch].cancel(false);
                off_[ch] = null;
            }
            final int c = ch;
            io_.execute(new Runnable() {
                @Override
                public void run() {
                    write(c, false);
                }
            });
        }
    }

    private void write(int channel, boolean high) {
        if (!isAvailable()) {
            return;
        }
        try {
            setters_[channel].invoke(group_, Boolean.valueOf(high));
        } catch (Exception e) {
            // Nothing to recover: the next write tries again. Kept silent so a broken bus cannot flood the log.
        }
    }

    /**
     * Serve the remote commands (class doc) on UDP `port`, on a background thread, until close().
     * Returns false if the port could not be opened.
     */
    public boolean listen(int port) {
        try {
            socket_ = new DatagramSocket(port);
        } catch (SocketException e) {
            return false;
        }
        Thread t = new Thread(new Runnable() {
            @Override
            public void run() {
                byte[] buf = new byte[256];
                while (socket_ != null && !socket_.isClosed()) {
                    try {
                        DatagramPacket in = new DatagramPacket(buf, buf.length);
                        socket_.receive(in);
                        String reply = handle(new String(in.getData(), 0, in.getLength(), "US-ASCII"));
                        byte[] out = reply.getBytes("US-ASCII");
                        socket_.send(new DatagramPacket(out, out.length, in.getAddress(), in.getPort()));
                    } catch (Exception e) {
                        // closed by close(), or one bad packet: the loop condition decides
                    }
                }
            }
        }, "ttl-pulse-listener");
        t.setDaemon(true);
        t.start();
        return true;
    }

    /** One remote command -> its reply (class doc). Public so an app can feed it commands itself. */
    public String handle(String command) {
        String[] w = command.trim().toUpperCase(Locale.ROOT).split("\\s+");
        if (w.length == 0 || w[0].length() == 0) {
            return "ERR empty command";
        }
        if (w[0].equals("STATUS")) {
            return isAvailable() ? "OK available=true outputs=" + CHANNELS + " group=" + GROUP_CLASS
                                 : "ERR " + reason_;
        }
        if (w[0].equals("ALLOFF")) {
            allLow();
            return "OK ALLOFF";
        }
        if (w[0].equals("PULSE") && w.length == 3) {
            try {
                int ch = Integer.parseInt(w[1]);
                int ms = Integer.parseInt(w[2]);
                if (!isAvailable()) {
                    return "ERR " + reason_;
                }
                return pulse(ch, ms) ? "OK PULSE " + ch + " " + ms
                                     : "ERR output must be 1-" + CHANNELS + ", width " + MIN_WIDTH_MS + "-"
                                       + MAX_WIDTH_MS + " ms";
            } catch (NumberFormatException e) {
                return "ERR PULSE <output> <width ms> takes two integers";
            }
        }
        return "ERR unknown command (PULSE <output> <width ms> | ALLOFF | STATUS)";
    }

    /** Stop listening, set every output low and release the I/O thread. */
    public void close() {
        DatagramSocket s = socket_;
        socket_ = null;
        if (s != null) {
            s.close();
        }
        allLow();
        io_.shutdown();
        try {
            io_.awaitTermination(200, TimeUnit.MILLISECONDS);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
