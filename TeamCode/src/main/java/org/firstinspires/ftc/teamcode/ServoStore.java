package org.firstinspires.ftc.teamcode;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.util.Locale;
import java.util.Map;
import java.util.TreeMap;

/** Servo "open / closed" numbers saved on the Hub: /sdcard/FIRST/car1_servo.txt, one line each: "name open|closed number". Car1HardwareTest writes it, Car1TeleOp reads it. */
public final class ServoStore {
    public static final String PATH = "/sdcard/FIRST/car1_servo.txt";
    private static final Map<String, Double> MAP = new TreeMap<>();
    public static String lastError = null;
    private static boolean loaded = false;

    private ServoStore() {}

    private static String key(String name, boolean open) { return name + " " + (open ? "open" : "closed"); }

    public static synchronized void load() {
        MAP.clear(); loaded = true;
        File f = new File(PATH);
        if (!f.exists()) return;
        try (BufferedReader r = new BufferedReader(new FileReader(f))) {
            String line;
            while ((line = r.readLine()) != null) {
                String[] p = line.trim().split(" +");
                if (p.length == 3) MAP.put(p[0] + " " + p[1], Double.parseDouble(p[2]));
            }
        } catch (Exception e) { lastError = e.toString(); }
    }

    public static synchronized double get(String name, boolean open, double dflt) {
        if (!loaded) load();
        Double v = MAP.get(key(name, open));
        return v == null ? dflt : v;
    }

    public static synchronized String fmt(String name, boolean open) {
        if (!loaded) load();
        Double v = MAP.get(key(name, open));
        return v == null ? "not saved" : String.format(Locale.US, "%.2f", v);
    }

    public static synchronized void save(String name, boolean open, double pos) {
        if (!loaded) load();
        MAP.put(key(name, open), pos);
        try (FileWriter w = new FileWriter(PATH, false)) {
            for (Map.Entry<String, Double> e : MAP.entrySet())
                w.write(String.format(Locale.US, "%s %.3f%n", e.getKey(), e.getValue()));
            lastError = null;
        } catch (Exception e) { lastError = e.toString(); }
    }
}
