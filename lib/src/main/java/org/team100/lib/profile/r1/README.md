# lib.profile.r1

This package contains one-dimensional profiles.

There are methods for scaling these profiles so they can be
coordinated to complete in the same duration.

## How to select a profile

The choices here vary in complexity and fidelity:

* `TrapezoidProfileWPI` wraps the WPI code: it's the simplest, with only two parameters.
* `TrapezoidIncrementalProfile` is similar, but implements moving end states correctly (the WPI version does not).
* `ExponentialProfileWPI` wraps the WPI exponential model, which represents motor behavior in the non-current-limited regime (i.e. where back EMF is the limiter).
* `CurrentLimitedExponentialProfile` chooses either a trapezoid, or exponential, depending on the initial speed: slower is current limited (trapezoid), faster is back-EMF limited (exponential).
* `CompleteProfile` is more complicated, supporting jerk limiting on takeoff and landing, distinct acceleration and deceleration limits, current limiting, and back-EMF limiting.  It only works for stationary goals, which is our most common case.