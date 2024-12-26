# all25

[![CI](https://github.com/Team100/all25/actions/workflows/main.yml/badge.svg)](https://github.com/Team100/all25/actions/workflows/main.yml)
[![CI](https://github.com/Team100/all25/actions/workflows/lib.yml/badge.svg)](https://github.com/Team100/all25/actions/workflows/lib.yml)
[![CI](https://github.com/Team100/all25/actions/workflows/raspberry_pi.yml/badge.svg)](https://github.com/Team100/all25/actions/workflows/raspberry_pi.yml)


all25 contains *all* the Team 100 code for *2025.*  There's just one repository,
so that sharing library code across projects is easier.

This style of code management is sometimes called a ["monorepo"](https://en.wikipedia.org/wiki/Monorepo) if you want to read more about it.

Here is the directory layout:

* **lib**: evergreen RoboRIO library code
* **comp**: RoboRIO competition code for 2025
* **raspberry_pi** coprocessor tasks: vision, logging, GTSAM
* **studies**: small independent projects
  * your project
  * another project
  * etc ...

# Getting Started

First time working on Control? Fret not! This guide will walk you through everything from setting up your development environment, to deploying code to the RoboRIO, to controlling your first motor, and then to building a complex robot. It's an in-progress guide so please add to it anytime you see gaps.

Chapters:
1. [Setting up your development environment](README_1_STARTING.md)
2. [Getting your first motor running](README_2_MOTOR.md)
3. [Creating a new 'study'](README_3_STUDY.md)

