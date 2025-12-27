# se2

Multi-dimensional profiled motion.

The main interface is `ProfileSE2`.  

Implementations include

* `HolonomicProfile` wraps any set of three `IncrementalProfile`s and coordinates them.
* `FreeRotationProfile` is similar but doesn't coordinate theta.