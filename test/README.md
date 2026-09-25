# Test

Full projects for mechanism testing.

These are intended for prototyping, when you want to run a minimum
amount of code focused on a single subsystem which has a single
degree of freedom.  Each example demonstrates usage and bindings
for a Team 100 "subsystem".

There are "single" and "dual" motor examples of each.  Sometimes we
run a light mechanism with a single motor, and sometimes we use two
in parallel, if we need more power than a single motor can provide.

There are three types of subsystems here:

## Linear

A linear subsystem is appropriate for something like an elevator
or a sliding joint.  We also use the "linear" type for things that
might seem rotary, but where the "interesting" thing is
actually linear position, for example wheels on the floor, or a
conveyor belt where position is important.

## Angular

An angular subsystem is appropriate for something like an arm.  For
example, something that swings out, where we want positional control.

## Roller

The roller subsystem supports velocity control, and it is common
where position doesn't matter, e.g. for intake rollers or shooter
drums.

# How to use this code

Each project has a single interesting file, `Robot.java`.  In there,
you'll find three interesting sections: configuration, instantiation, and binding.

## Configuration

In `Robot.java`, you'll find a section that looks something like this:

```java
        // CONFIGURATION

        CanId canId1 = new CanId(1);
        NeutralMode100 neutral = NeutralMode100.COAST;
        MotorPhase phase1 = MotorPhase.FORWARD;
        CurrentLimit limit = new CurrentLimit(1, 1);
        double gearRatio = 6.0;
        double wheelDiameterM = 0.025;
        Friction friction = new Friction(0.32, 0.32, 0.0, 0.5);
        PIDConstants pid = PIDConstants.makePositionPID(1);
        PDynamics dynamics = new PDynamics(1);
        double xtolerance = 0.01;
        double vtolerance = 0.01;
        TrapezoidProfileR1 profile = new TrapezoidProfileR1(0.1, 0.25, xtolerance);
        ReferenceR1 ref = new ProfileReferenceR1(log, () -> profile, xtolerance, vtolerance);
```

Each of these parameters affects some part of the subsystem.  The most important one is
the CAN id, which you'll need to get from TunerX, or from the physical writing on the
motor.  If you want to save a particular set of test values, you can make a copy
of `Robot.java` or the entire project, and check it in.

## Instantiation

Each project uses just a single subsystem class, so the instantiation in `Robot.java` is just
one line, for example:

```java
        // SUBSYSTEM

        m_subsystem = new SingleLinearSubsystem(
                log, currentLog, canId1, neutral, phase1, limit,
                friction, pid, gearRatio, wheelDiameterM, dynamics, ref, xtolerance, vtolerance, true);
```

All the parameters here come from the configuration section.

## Bindings

The controller button bindings are the last part of the `Robot.java` constructor, for example,
this shows positional control with presets, zeroing, and velocity control:

```java
        // BINDINGS

        m_subsystem.setDefaultCommand(m_subsystem.stop().withName("stop"));
        m_control = new DriverXboxControl(log, 0);
        whileTrue(m_control::start, m_subsystem.zero().withName("zero"));
        whileTrue(m_control::a, m_subsystem.position(0).withName("in"));
        whileTrue(m_control::b, m_subsystem.position(0.1).withName("out"));
        whileTrue(m_control::leftBumper, m_subsystem.velocity(-0.1).withName("negative"));
        whileTrue(m_control::rightBumper, m_subsystem.velocity(0.1).withName("positive"));
```

Feel free to change the bindings and/or the values in them, if you like.  If you want to
do anything more complicated than these basic "turn up" actions, you should make a "study".
It's ok to cut-and-paste the `Robot.java` file from this test area into your study, as
a starting point.
