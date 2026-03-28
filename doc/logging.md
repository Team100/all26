# Logging

## Upstream WPILib Logging

WPILib provides several layers for telemetry and data logging.

### DataLog / DataLogManager

The core disk-logging system. `DataLogManager.start()` begins writing binary `.wpilog` files to a USB stick (preferred) or `/home/lvuser/logs` on the roboRIO. All file I/O happens on a background thread, so robot code only pays for a mutex and a memcpy.

Key behaviors:

- Automatically captures all NetworkTables value changes (toggleable via `logNetworkTables(false)`)
- Captures console output (toggleable via `logConsoleOutput(false)`)
- Logs system time every ~5 seconds
- Files are named `FRC_yyyyMMdd_HHmmss.wpilog` once the Driver Station connects; at competition with FMS they include event and match info
- Old files without a DS connection are cleaned up on startup; oldest files are purged when free space drops below 50 MB

### NetworkTables

The real-time pub/sub system. Anything published to NT is visible in live dashboards (Glass, Shuffleboard, AdvantageScope). With `DataLogManager` running, NT changes are automatically mirrored to disk.

### Typed LogEntry Classes

`BooleanLogEntry`, `DoubleLogEntry`, `IntegerLogEntry`, `StringLogEntry`, `DoubleArrayLogEntry`, etc. You create one per data series and call `.append(value)` each cycle. Unlike NetworkTables, there is no change detection -- every `append()` writes a record.

### Replay

`.wpilog` files are opened in AdvantageScope for timeline visualization, plotting, and 3D field rendering. The simpler DataLogTool utility can also view and convert log files. For programmatic access, `DataLogReader` is available in Java, C++, and Python.

### Epilogue / @Logged (2025+)

WPILib introduced an annotation-processor system (`edu.wpi.first.epilogue`) that auto-generates logging code for annotated classes. This project does **not** use Epilogue -- it has its own framework instead.

### Upstream Documentation

- [On-Robot Telemetry Recording Into Data Logs](https://docs.wpilib.org/en/stable/docs/software/telemetry/datalog.html)
- [Telemetry Overview](https://docs.wpilib.org/en/stable/docs/software/telemetry/telemetry.html)
- [Downloading & Processing Data Logs](https://docs.wpilib.org/en/stable/docs/software/telemetry/datalog-download.html)
- [Robot Telemetry with Annotations](https://docs.wpilib.org/en/stable/docs/software/telemetry/robot-telemetry-with-annotations.html)

---

## Team 100 Logging Framework

The project wraps WPILib's primitives in a custom framework under `lib/src/main/java/org/team100/lib/logging/`. See also the README at that path.

### Architecture Overview

```
SmartDashboard "Log Level" dropdown
        |
        v
  Logging singleton (isAllowed())  <-- FMS override: latches to COMP
        |
        v  Predicate<Level>
  LoggerFactory (root: "log" or "field")
        |
        v  .name() / .type()
  Child LoggerFactory (builds hierarchical paths)
        |
        v  .doubleLogger(Level.COMP, "velocity (m_s)")
  Typed Logger (e.g. DoubleLogger)
        |
        v  .log(() -> value)
  PrimitiveLogger interface
        |
        v
  NTPrimitiveLogger (publishes to NetworkTables)
        |
        v  (automatic via DataLogManager)
  .wpilog file on disk
```

### Key Classes

| Class | Path | Purpose |
|-------|------|---------|
| `Logging` | `lib/.../logging/Logging.java` | Singleton. Creates the NT backend, starts DataLogManager, exposes `rootLogger` and `fieldLogger`, hosts the level chooser. |
| `LoggerFactory` | `lib/.../logging/LoggerFactory.java` | Main API. Builds a hierarchical tree of named loggers. Provides 44+ typed logger inner classes. |
| `Level` | `lib/.../logging/Level.java` | Enum with three tiers: COMP, DEBUG, TRACE. |
| `PrimitiveLogger` | `lib/.../logging/primitive/PrimitiveLogger.java` | Transport abstraction interface. |
| `NTPrimitiveLogger` | `lib/.../logging/primitive/NTPrimitiveLogger.java` | Implementation that publishes to NetworkTables (retained topics) and starts DataLogManager for disk mirroring. |
| `RobotLog` | `lib/.../logging/RobotLog.java` | Logs robot-scope metrics: battery voltage, JVM memory, garbage collection. |
| `TotalCurrentLog` | `lib/.../logging/TotalCurrentLog.java` | Aggregates supply current from all registered motors. |
| `TestLoggerFactory` | `lib/.../logging/TestLoggerFactory.java` | Test utility that forces TRACE level. |

### Log Levels

```java
public enum Level {
    COMP(1),    // Minimal, curated set for competition matches
    DEBUG(2),   // Things actively being worked on (keep small to avoid overruns)
    TRACE(3);   // Everything (slow, causes loop overruns)
}
```

### Level Control

The level is selected at runtime via a SmartDashboard dropdown labeled "Log Level". The default is `DEBUG`.

The `Logging` singleton creates a `SendableChooser<Level>` in a static initializer and publishes it to SmartDashboard. It passes `this::isAllowed` (a `Predicate<Level>`) into every `LoggerFactory`. All child factories created via `.name()` or `.type()` inherit the same predicate, so **all loggers share one global level** -- changing the dropdown takes effect on the next loop iteration.

#### FMS Auto-Switch

When the robot is connected to the Field Management System (i.e. during a real competition match), the level is automatically forced to `COMP`. This is implemented as a latch in `Logging.refreshLogLevel()`, which is called by `IterativeRobotBase100` every loop iteration (50Hz):

- If `DriverStation.isFMSAttached()` returns true, the level is latched to `COMP` permanently for the session.
- Once latched, subsequent calls short-circuit on a single volatile read -- no `SendableChooser` lock acquired.
- When FMS is not attached (practice, pit testing), the dropdown works normally and changes take effect on the next iteration.

The precomputed `boolean[]` allow cache eliminates ~14,000 `SendableChooser.getSelected()` calls per second (one per logger per cycle), each of which previously acquired a `ReentrantLock`.

#### Gating Logic

Each logger's `.log()` method calls `allow()` before evaluating the supplier:

```java
private boolean allow(Level level) {
    return m_allow.test(level);
}
```

`Level.admit()` compares integer priorities:

```java
public boolean admit(Level other) {
    return this.priority >= other.priority;  // TRACE(3) admits everything, COMP(1) admits only COMP
}
```

| Dashboard selection | What gets logged |
|---|---|
| **COMP** | Only `Level.COMP` entries |
| **DEBUG** | `COMP` + `DEBUG` entries |
| **TRACE** | Everything (expect loop overruns) |

#### The Supplier Trick

Because `.log()` takes a `Supplier` / `DoubleSupplier` rather than a raw value, expensive computation (e.g. a CAN bus read) is **never executed** when the level is filtered out:

```java
public void log(DoubleSupplier vals) {
    if (!allow(m_level))
        return;                    // supplier never called
    double val = vals.getAsDouble();
    m_primitiveLogger.log(val);
}
```

### Usage Pattern

Loggers are created in constructors and passed down the object tree. Each class makes a child factory, so the logger tree mirrors instantiation:

```java
// In a subsystem constructor
public Conveyor(LoggerFactory parent, TotalCurrentLog currentLog) {
    LoggerFactory log = parent.type(this);        // "log/.../Conveyor"
    LoggerFactory log1 = log.name("Conveyor1");   // "log/.../Conveyor/Conveyor1"

    // Motors create their own child loggers from log1
    m1 = new KrakenX44Motor(log1, currentLog, ...);
}
```

Creating typed loggers at specific levels:

```java
// COMP level -- always logged in competition
m_log_output = m_log.doubleLogger(Level.COMP, "output [-1,1]");
m_log_position = m_log.doubleLogger(Level.COMP, "position (rad)");

// DEBUG level -- logged when debugging
m_log_desired_speed = m_log.doubleLogger(Level.DEBUG, "desired speed (rad_s)");

// TRACE level -- expensive, detailed info
m_log_desired_accel = m_log.doubleLogger(Level.TRACE, "desired accel (rad_s2)");
m_log_friction_FF = m_log.doubleLogger(Level.TRACE, "friction feedforward (V)");
```

Logging values with suppliers:

```java
m_log_desired_speed.log(() -> motorRad_S);
m_log_friction_FF.log(() -> frictionFFVolts);
```

### What Gets Logged

| Level | Examples |
|-------|----------|
| **COMP** | Battery voltage, total supply current, motor output/position, servo goals, match time, autonomous/teleop mode flags, FMS attached |
| **DEBUG** | Desired speeds, control errors, pose estimates, supply/stator current, motor temperature, heap memory |
| **TRACE** | Feedforward terms (friction, torque), acceleration targets, JVM non-heap memory, GC stats, gyro angles, raw sensor voltages |

### Dual Output

`NTPrimitiveLogger` publishes to NetworkTables with retained topics (so values persist after logging stops and are visible in live dashboards). `DataLogManager` automatically mirrors all NT changes to binary `.wpilog` files on disk. This gives both real-time dashboard viewing and persistent logs for post-match analysis.

### Additional Details

- CTRE's built-in `SignalLogger` auto-logging is disabled (`SignalLogger.enableAutoLogging(false)`) to avoid redundant overhead.
- The `fieldLogger` factory (root: `"field"`) publishes a `.type` = `"Field2d"` entry so that Glass renders it as a field widget.
- Don't use slashes in logger names -- it confuses Glass.
