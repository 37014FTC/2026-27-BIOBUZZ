# ServoStore.java — line by line

**File:** `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/ServoStore.java` (56 lines)
**What it is:** a small helper (not an OpMode) that saves servo "open" and "closed" positions in a text file on the Control Hub, so numbers tuned in *Car1 Hardware Test* are automatically used by *Car1 TeleOp* and the autos on the next run.
**Big idea:** servos don't have sensors telling you where they are — the code just sends a number between 0.0 and 1.0. The perfect "open" and "closed" numbers are found by experimenting with the real robot, so they are stored on the Hub's storage (`/sdcard/FIRST/`), which survives a program restart.

---

## Lines 1–9 — package and imports

```java
package org.firstinspires.ftc.teamcode;
```
- **Line 1** — Every Java file starts with a `package` line: the "folder address" of this class. All team code lives in `org.firstinspires.ftc.teamcode`, which is why the file must sit in `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`.

```java
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.util.Locale;
import java.util.Map;
import java.util.TreeMap;
```
- **Lines 3–9** — `import` statements are a shopping list of tools from other libraries that this file wants to use, so you can write short names (`File`) instead of full addresses (`java.io.File`).
  - `BufferedReader` / `FileReader` — tools for reading a text file line by line.
  - `FileWriter` — tool for writing text to a file.
  - `File` — represents a file on disk (used here just to ask "does it exist?").
  - `Locale` — used below to force the decimal point to be a dot.
  - `Map` / `TreeMap` — a dictionary: "key → value" pairs, here `"sorterGateR open" → 0.46`. `TreeMap` keeps the keys sorted alphabetically, so the saved file always has the same order.

## Line 11 — the class comment

```java
/** Servo "open / closed" numbers saved on the Hub: /sdcard/FIRST/car1_servo.txt, one line each: "name open|closed number". Car1HardwareTest writes it, Car1TeleOp reads it. */
```
- **Line 11** — A `/** ... */` comment is a Javadoc comment: it documents the class for humans. It also defines the file format: each line looks like `sorterGateL open 0.900`.

## Lines 12–18 — the class and its static fields

```java
public final class ServoStore {
```
- **Line 12** — Declares the class. `public` = usable from anywhere; `final` = no other class may extend/modify it (it's a finished utility); `ServoStore` = the class name, matching the file name.

```java
    public static final String PATH = "/sdcard/FIRST/car1_servo.txt";
```
- **Line 13** — The full file path on the Control Hub's internal storage. `static` = belongs to the class itself, not to one object; `final` = a constant that can never be reassigned.

```java
    private static final Map<String, Double> MAP = new TreeMap<>();
```
- **Line 14** — The in-memory copy of the file: keys are text like `"sorterGateL open"`, values are the saved positions. `<>` means "the value type I already said on the left" (Java can fill it in).

```java
    public static String lastError = null;
```
- **Line 15** — If reading or writing the file fails, the error message is parked here so the OpMode can show it on the Driver Station. `null` means "no error yet".

```java
    private static boolean loaded = false;
```
- **Line 16** — Remembers whether the file has been read at least once, so the first `get()` call can load it automatically (called *lazy loading*).

```java
    private ServoStore() {}
```
- **Line 18** — A private constructor makes it impossible to create a `ServoStore` object — everything is used statically, like `ServoStore.get(...)`. This is the standard trick for "utility classes" (same idea as `Math` — you write `Math.cos(...)`, never `new Math()`).

## Line 20 — key builder

```java
    private static String key(String name, boolean open) { return name + " " + (open ? "open" : "closed"); }
```
- **Line 20** — Turns a servo name plus an open/closed flag into the dictionary key, e.g. `("sorterGateR", true)` → `"sorterGateR open"`. The `(open ? "open" : "closed")` is a *ternary*: "if open then "open" else "closed"". Writing it once here guarantees every method builds keys the same way.

## Lines 22–33 — load(): read the file into MAP

```java
    public static synchronized void load() {
```
- **Line 22** — `synchronized` means only one thread can run this method at a time (a safety lock — harmless here, good habit).

```java
        MAP.clear(); loaded = true;
```
- **Line 23** — Empty the dictionary (so reloading gives fresh data) and mark "file has been read".

```java
        File f = new File(PATH);
        if (!f.exists()) return;
```
- **Lines 24–25** — If the file doesn't exist yet (first ever run), leave now with an empty MAP. Callers fall back to the numbers built into the code.

```java
        try (BufferedReader r = new BufferedReader(new FileReader(f))) {
```
- **Line 26** — Opens the file for reading. The `try (... )` form is *try-with-resources*: whatever happens, Java automatically closes the file afterwards — no leaked file handles.

```java
            String line;
            while ((line = r.readLine()) != null) {
```
- **Lines 27–28** — Read one line at a time until `readLine()` returns `null` (end of file). The double use of `line` (assign inside the test) is the classic Java file-reading pattern.

```java
                String[] p = line.trim().split(" +");
```
- **Line 29** — `trim()` removes spaces at both ends; `split(" +")` cuts the line at each run of one-or-more spaces. `"sorterGateL open 0.900"` becomes `["sorterGateL", "open", "0.900"]`.

```java
                if (p.length == 3) MAP.put(p[0] + " " + p[1], Double.parseDouble(p[2]));
```
- **Line 30** — Only lines with exactly 3 pieces are accepted. The first two pieces are joined back into the key; `Double.parseDouble` converts the text `"0.900"` into the number `0.900` and stores it.

```java
        } catch (Exception e) { lastError = e.toString(); }
```
- **Line 32** — If anything goes wrong (unreadable file, a piece that isn't a number), don't crash the robot program — just remember the error message so it can be shown on screen.

```java
    }
```
- **Line 33** — End of `load()`.

## Lines 35–39 — get(): read one value with a fallback

```java
    public static synchronized double get(String name, boolean open, double dflt) {
        if (!loaded) load();
        Double v = MAP.get(key(name, open));
        return v == null ? dflt : v;
    }
```
- **Line 35** — Signature: give a servo name, say whether you want its OPEN or CLOSED value, and give a default for "nothing saved". Returns the number.
- **Line 36** — First ever call? Read the file now (lazy loading).
- **Line 37** — Look up the value; result is `null` when nothing was saved for that key.
- **Line 38** — Ternary again: saved value if there is one, otherwise the caller's default. This is why TeleOp works even before anyone has ever saved anything.

## Lines 41–45 — fmt(): show the value on screen

```java
    public static synchronized String fmt(String name, boolean open) {
        if (!loaded) load();
        Double v = MAP.get(key(name, open));
        return v == null ? "not saved" : String.format(Locale.US, "%.2f", v);
    }
```
- **Line 41** — Same as `get` but returns display text for telemetry.
- **Line 42** — Lazy-load like before.
- **Line 43** — Look up the value.
- **Line 44** — `String.format(Locale.US, "%.2f", v)` turns `0.9` into `"0.90"`. `%.2f` = "decimal number with 2 digits after the point"; `Locale.US` forces a dot as the decimal mark (some phones would otherwise print `0,90` with a comma, depending on language settings — a classic FTC bug this line prevents).

## Lines 47–55 — save(): change one value and rewrite the whole file

```java
    public static synchronized void save(String name, boolean open, double pos) {
        if (!loaded) load();
        MAP.put(key(name, open), pos);
```
- **Line 47** — Called by Hardware Test when you press Y (save open) or A (save closed) with a servo selected.
- **Line 48** — Lazy-load first (so you don't erase values saved earlier that aren't in memory yet).
- **Line 49** — Put the new value into the dictionary, replacing any old one.

```java
        try (FileWriter w = new FileWriter(PATH, false)) {
```
- **Line 50** — Open the file for writing. The second argument `false` means *overwrite* the whole file (not append). Auto-closed by try-with-resources.

```java
            for (Map.Entry<String, Double> e : MAP.entrySet())
                w.write(String.format(Locale.US, "%s %.3f%n", e.getKey(), e.getValue()));
```
- **Lines 51–52** — Walk every entry in the (alphabetically sorted) dictionary and write one line per entry. `%s` = the key text, `%.3f` = the number with 3 decimals, `%n` = a newline.

```java
            lastError = null;
```
- **Line 53** — Saving worked, so clear any old error.

```java
        } catch (Exception e) { lastError = e.toString(); }
```
- **Line 54** — Any problem (e.g. disk full) is stored instead of crashing.

## Line 56

```java
}
```
- **Line 56** — End of the class.

---

## Evaluation

**Strengths**
- Solves a real rookie-team problem well: servo numbers survive restarts without recompiling.
- Never crashes the robot program — every file problem becomes a visible message, and every lookup has a built-in default.
- `Locale.US` formatting avoids the comma-decimal bug.
- Rewriting the whole (sorted) file on each save keeps the format simple and the file human-readable.

**Watch-outs (nothing changed, just things to know)**
- `catch` wraps the whole read loop (line 26/32): one malformed line stops the rest of the file from being read. Entries before it survive — fine in practice.
- `lastError` is shared by read and write, so a read error stays on screen until a successful write clears it.
