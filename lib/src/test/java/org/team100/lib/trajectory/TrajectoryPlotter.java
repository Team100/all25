package org.team100.lib.trajectory;

import java.awt.Dimension;
import java.awt.Frame;
import java.util.List;
import java.util.function.Supplier;

import javax.swing.BoxLayout;
import javax.swing.JDialog;
import javax.swing.WindowConstants;

import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.axis.NumberAxis;
import org.jfree.chart.plot.XYPlot;
import org.jfree.chart.renderer.xy.VectorRenderer;
import org.jfree.chart.renderer.xy.XYItemRenderer;
import org.jfree.data.Range;
import org.jfree.data.xy.VectorSeries;
import org.jfree.data.xy.VectorSeriesCollection;
import org.jfree.data.xy.XYDataset;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;
import org.team100.lib.trajectory.path.Path100;
import org.team100.lib.trajectory.path.spline.HolonomicSpline;
import org.team100.lib.trajectory.path.spline.SplineToVectorSeries;

public class TrajectoryPlotter {
    public static final boolean SHOW = false;
    private static final int SIZE = 500;

    private final TrajectoryToVectorSeries converter;
    private final SplineToVectorSeries splineConverter;
    private final PathToVectorSeries pathConverter;

    public TrajectoryPlotter(double arrowLength) {
        converter = new TrajectoryToVectorSeries(arrowLength);
        splineConverter = new SplineToVectorSeries(arrowLength);
        pathConverter = new PathToVectorSeries(arrowLength);
    }

    public void plot(String name, Trajectory100 t) {
        VectorSeries converted = convert(name, t);
        XYDataset dataSet = TrajectoryPlotter.collect(converted);
        if (SHOW)
            actuallyPlot(name, () -> new VectorRenderer(), dataSet);
    }

    public VectorSeries convert(String name, Trajectory100 t) {
        return converter.convert(name, t);
    }

    public VectorSeries convert(String name, Path100 path) {
        return pathConverter.convert(name, path);
    }

    public void plot(String name, List<HolonomicSpline> s) {
        VectorSeries converted = convert(name, s);
        XYDataset dataSet = TrajectoryPlotter.collect(converted);
        if (SHOW)
            actuallyPlot(name, () -> new VectorRenderer(), dataSet);
    }

    public VectorSeries convert(String name, List<HolonomicSpline> s) {
        return splineConverter.convert(name, s);
    }

    public static XYDataset collect(VectorSeries... converted) {
        VectorSeriesCollection dataSet = new VectorSeriesCollection();
        for (VectorSeries c : converted) {
            dataSet.addSeries(c);
        }
        return dataSet;
    }

    public static XYDataset collect(XYSeries... series) {
        XYSeriesCollection dataSet = new XYSeriesCollection();
        for (XYSeries c : series) {
            dataSet.addSeries(c);
        }
        return dataSet;
    }

    public static Range xRange(XYDataset dataset) {
        double max = Double.NEGATIVE_INFINITY;
        double min = Double.POSITIVE_INFINITY;
        for (int i = 0; i < dataset.getItemCount(0); ++i) {
            double x = dataset.getXValue(0, i);
            if (x + 0.1 > max)
                max = x + 0.1;
            if (x - 0.1 < min)
                min = x - 0.1;
        }
        return new Range(min, max);
    }

    public static Range yRange(XYDataset dataset) {
        double max = Double.NEGATIVE_INFINITY;
        double min = Double.POSITIVE_INFINITY;
        for (int i = 0; i < dataset.getItemCount(0); ++i) {
            double y = dataset.getYValue(0, i);
            if (y + 0.1 > max)
                max = y + 0.1;
            if (y - 0.1 < min)
                min = y - 0.1;
        }
        return new Range(min, max);
    }

    /**
     * renderer is a supplier because new XYPlot writes the dataset name into the
     * renderer.
     * 
     * mutability is bad.
     */
    public static void actuallyPlot(
            String name,
            Supplier<XYItemRenderer> renderer,
            XYDataset... dataSets) {
        if (!SHOW)
            return;

        // "true" means "modal" so wait for close.
        JDialog frame = new JDialog((Frame) null, name, true);
        frame.setLayout(new BoxLayout(frame.getContentPane(), BoxLayout.Y_AXIS));

        for (XYDataset dataSet : dataSets) {
            XYPlot plot = new XYPlot(
                    dataSet,
                    new NumberAxis("X"),
                    new NumberAxis("Y"),
                    renderer.get());
            NumberAxis domain = (NumberAxis) plot.getDomainAxis();
            NumberAxis range = (NumberAxis) plot.getRangeAxis();
            domain.setRangeWithMargins(xRange(dataSet));
            range.setRangeWithMargins(yRange(dataSet));

            ChartPanel panel = new ChartPanel(new JFreeChart(plot));
            panel.setPreferredSize(new Dimension(SIZE, SIZE / dataSets.length));
            frame.add(panel);
        }

        frame.pack();
        frame.setVisible(true);
        frame.setDefaultCloseOperation(WindowConstants.DISPOSE_ON_CLOSE);
    }

    public static void plot(
            Trajectory100 t,
            double arrows) {
        TrajectoryPlotter plotter = new TrajectoryPlotter(arrows);
        VectorSeries series = plotter.convert("trajectory", t);
        XYDataset dataSet = TrajectoryPlotter.collect(series);
        if (SHOW)
            actuallyPlot("compare", () -> new VectorRenderer(), dataSet);
    }

    public static void plot(
            List<HolonomicSpline> splines,
            double arrows) {
        TrajectoryPlotter plotter = new TrajectoryPlotter(arrows);
        VectorSeries series = plotter.convert("before", splines);
        XYDataset dataSet = TrajectoryPlotter.collect(series);
        if (SHOW)
            actuallyPlot("compare", () -> new VectorRenderer(), dataSet);
    }

    public static void plot(
            Path100 path,
            double arrows) {
        TrajectoryPlotter plotter = new TrajectoryPlotter(arrows);
        VectorSeries series = plotter.convert("path", path);
        XYDataset dataSet = TrajectoryPlotter.collect(series);
        if (SHOW)
            actuallyPlot("compare", () -> new VectorRenderer(), dataSet);
    }
}
