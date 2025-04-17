# Conventions

Team 100 code uses a number of conventions in style and method.

## Style

We mirror WPILib naming conventions:

* members start with "m_", so, ```double m_thing;```
* use the minimum appropriate visibility.  for most members, this is ```private.```
* use ```final``` liberally.  most members should be ```final```.  favor immutability in general: it saves you from mysteries at runtime.

The most important aspect of Team 100 code is simplicity.  Classes should be small.  Methods should be short.

Use names that describe the meaning of what you're doing.  If you change the meaning, change the name.  If the meaning is really obvious, it's ok to use really short names like ```x```.

## Infrastructure

A common pattern is to print stuff to the console, if some sort of "debug" mode is engaged.  There are currently two patterns for that, a static boolean, or an interface that includes the printing methods -- the latter is preferred.

## Branching

If you know how to do it, it's encouraged to work on a branch in your fork, so that you can switch branches to work on more than one thing at a time.  If you don't know how to do it, don't worry about it.