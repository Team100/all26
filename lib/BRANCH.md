# 2027-GTSAM

This branch exists to explore using GTSAM with real robot code on the Systemcore.

# building

if you're building vendordep locally, you probably copied it to the local maven:

`~/releases/maven/development/org/team100/gtsam-vendordep`

so you could copy it:

```
cp -r releases/maven/development/org/team100/gtsam-vendordep wpilib/2027_alpha5/maven/org/team100/
```

# building from maven online

I put the maven artifacts in my repo:

`https://github.com/truher/repo/tree/main/org/team100/gtsam-vendordep`


and from my desktop, they seem to be fetched correctly by the vendordep json.