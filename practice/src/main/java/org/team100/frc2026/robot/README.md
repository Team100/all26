# robot

The reason for the robot package is so that we can break up the large amount
of initialization we need to do into separate files, to reduce merge conflicts.

The general idea is that all the mechanical stuff in the robot goes in
`Machinery`, all the trigger bindings go in `Binder`, and all the
autonomous behaviors go in `AllAutons`.

From `Robot.java`:

```java
        m_machinery = new Machinery();
        m_allAutons = new AllAutons(m_machinery);
        m_binder = new Binder(m_machinery);
        m_binder.bind();
```


