function normPoint = getNormalizedCoordinates(point, ax)
    % Convert to normalized figure units
    pt = ax.Position;
    xLimits = ax.XLim;
    yLimits = ax.YLim;
    normPoint(1) = pt(1) + (point(1) - xLimits(1)) / diff(xLimits) * pt(3);
    normPoint(2) = pt(2) + (point(2) - yLimits(1)) / diff(yLimits) * pt(4);
end