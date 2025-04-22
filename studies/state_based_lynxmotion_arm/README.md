# State based Lynxmotion arm

This studies the use of statecharts to organize robot behavior.

https://wempe.dev/blog/what-are-state-machines-and-statecharts

The general idea is to create a directed graph of robot states.

The robot can be "in" one state at a time.

Each state is represented by a trigger on the matching state.

You can attach commands to each state, with or without guard conditions.

You can also attach transitions to each state, using observations.