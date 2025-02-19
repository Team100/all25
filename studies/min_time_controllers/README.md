# Minimum Time Controllers

The minimum-time controller is uses maximum effort all the time:
speed up as fast as possible, slow down as possible.

It's a type of "sliding mode" controller that uses a trapezoidal profile.

Like all sliding-mode controllers, it chatters, and I haven't found time
to really make it work well.

In reality we'd want to combine it with some sort of "easing" at the end
(it already does this to some extent, with a little "PID zone" near the
target).

If you want to learn more about sliding mode controllers, or the minimum
time controller, ask Joel about it.