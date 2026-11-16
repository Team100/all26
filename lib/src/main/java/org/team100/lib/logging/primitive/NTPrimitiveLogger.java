package org.team100.lib.logging.primitive;

import java.util.HashSet;
import java.util.Set;

import org.wpilib.networktables.BooleanPublisher;
import org.wpilib.networktables.BooleanTopic;
import org.wpilib.networktables.DoubleArrayPublisher;
import org.wpilib.networktables.DoubleArrayTopic;
import org.wpilib.networktables.DoublePublisher;
import org.wpilib.networktables.DoubleTopic;
import org.wpilib.networktables.IntegerPublisher;
import org.wpilib.networktables.IntegerTopic;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.StringPublisher;
import org.wpilib.networktables.StringTopic;
import org.wpilib.networktables.StructArrayPublisher;
import org.wpilib.networktables.StructArrayTopic;
import org.wpilib.networktables.StructPublisher;
import org.wpilib.networktables.StructTopic;
import org.wpilib.system.DataLogManager;
import org.wpilib.util.struct.Struct;

/**
 * This is a sort of back-up option for logging directly to a local network
 * tables server.
 * 
 * Logged items are "retained" which means they persist even after the logging
 * stops; this means you should see the latest values after disabling the robot.
 * 
 * With a full load of logging this will overrun the 50hz loop very badly, so if
 * you use it, you'll need to turn off most of the logging.
 */
public class NTPrimitiveLogger implements PrimitiveLogger {
    // for hunting the field2d issue
    private static final boolean RETAIN = false;
    private final NetworkTableInstance inst;
    // this is duplicative of the NT topic list, but the NT topics includes
    // other non-logging keys, so we keep our own list.
    private final Set<String> keys = new HashSet<>();

    public NTPrimitiveLogger() {
        NetworkTableInstance.getDefault().startServer();
        inst = NetworkTableInstance.getDefault();
        // Also log to disk
        DataLogManager.start();
    }

    @Override
    public int keyCount() {
        return keys.size();
    }

    public class NTBooleanLogger implements PrimitiveLogger.PrimitiveBooleanLogger {
        BooleanPublisher m_pub;

        public NTBooleanLogger(String label) {
            BooleanTopic t = inst.getBooleanTopic(label);
            m_pub = t.publish();
            t.setRetained(RETAIN);
        }

        @Override
        public void log(boolean val) {
            m_pub.set(val);
        }

    }

    public class NTDoubleLogger implements PrimitiveLogger.PrimitiveDoubleLogger {
        DoublePublisher m_pub;

        public NTDoubleLogger(String label) {
            DoubleTopic t = inst.getDoubleTopic(label);
            m_pub = t.publish();
            t.setRetained(RETAIN);
        }

        @Override
        public void log(double val) {
            m_pub.set(val);
        }
    }

    public class NTIntLogger implements PrimitiveLogger.PrimitiveIntLogger {
        IntegerPublisher m_pub;

        public NTIntLogger(String label) {
            IntegerTopic t = inst.getIntegerTopic(label);
            m_pub = t.publish();
            t.setRetained(RETAIN);
        }

        @Override
        public void log(int val) {
            m_pub.set(val);
        }
    }

    public class NTDoubleArrayLogger implements PrimitiveLogger.PrimitiveDoubleArrayLogger {
        DoubleArrayPublisher m_pub;

        public NTDoubleArrayLogger(String label) {
            DoubleArrayTopic t = inst.getDoubleArrayTopic(label);
            m_pub = t.publish();
            t.setRetained(RETAIN);
        }

        @Override
        public void log(double[] val) {
            m_pub.set(val);
        }
    }

    public class NTLongLogger implements PrimitiveLogger.PrimitiveLongLogger {
        IntegerPublisher m_pub;

        public NTLongLogger(String label) {
            IntegerTopic t = inst.getIntegerTopic(label);
            m_pub = t.publish();
            t.setRetained(RETAIN);
        }

        @Override
        public void log(long val) {
            m_pub.set(val);
        }
    }

    public class NTStringLogger implements PrimitiveLogger.PrimitiveStringLogger {
        StringPublisher m_pub;

        public NTStringLogger(String label) {
            StringTopic t = inst.getStringTopic(label);
            m_pub = t.publish();
            t.setRetained(RETAIN);
        }

        @Override
        public void log(String val) {
            m_pub.set(val);
        }
    }

    public class NTStructLogger<T> implements PrimitiveLogger.PrimitiveStructLogger<T> {
        StructPublisher<T> m_pub;

        public NTStructLogger(String label, Struct<T> struct) {
            StructTopic<T> t = inst.getStructTopic(label, struct);
            m_pub = t.publish();
            t.setRetained(RETAIN);
        }

        @Override
        public void log(T val) {
            m_pub.set(val);
        }
    }

    public class NTStructArrayLogger<T> implements PrimitiveLogger.PrimitiveStructArrayLogger<T> {
        StructArrayPublisher<T> m_pub;

        public NTStructArrayLogger(String label, Struct<T> struct) {
            StructArrayTopic<T> t = inst.getStructArrayTopic(label, struct);
            m_pub = t.publish();
            t.setRetained(RETAIN);
        }

        @Override
        public void log(T[] val) {
            m_pub.set(val);
        }
    }

    @Override
    public PrimitiveBooleanLogger booleanLogger(String label) {
        keys.add(label);
        return new NTBooleanLogger(label);
    }

    @Override
    public PrimitiveDoubleLogger doubleLogger(String label) {
        keys.add(label);
        return new NTDoubleLogger(label);
    }

    @Override
    public PrimitiveIntLogger intLogger(String label) {
        keys.add(label);
        return new NTIntLogger(label);
    }

    @Override
    public PrimitiveDoubleArrayLogger doubleArrayLogger(String label) {
        keys.add(label);
        return new NTDoubleArrayLogger(label);
    }

    @Override
    public PrimitiveLongLogger longLogger(String label) {
        keys.add(label);
        return new NTLongLogger(label);
    }

    @Override
    public PrimitiveStringLogger stringLogger(String label) {
        keys.add(label);
        return new NTStringLogger(label);
    }

    @Override
    public <T> PrimitiveStructLogger<T> structLogger(String label, Struct<T> struct) {
        keys.add(label);
        return new NTStructLogger<>(label, struct);
    }

    @Override
    public <T> PrimitiveStructArrayLogger<T> structArrayLogger(String label, Struct<T> struct) {
        keys.add(label);
        return new NTStructArrayLogger<>(label, struct);
    }

}
