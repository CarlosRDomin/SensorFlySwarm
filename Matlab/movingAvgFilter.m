function out = movingAvgFilter(windowSize, in)
    a = 1;
    b = (1/windowSize)*ones(1,windowSize);
    out = filter(b, a, in);
end