package org.team100.lib.trajectory.timing;

/**
 * Resamples a trajectory with constant time between samples.
 * 
 * The idea is to prevent overworking the fast parts (where few samples are
 * required) and underworking the slow parts (where more samples are required),
 * as the spatial sampling tends to do. This allows the overall
 * trajectory-making process to be faster, but have higher resolution where it
 * matters.
 * 
 * The sampling goes all the way back to the source spline, to avoid
 * interpolation error, so the trajectory needs to include it.
 * 
 * The new samples are used to reschedule a new trajectory which may have a
 * different duration.
 */
public class TrajectoryRecycler {

}
