function out = constuctLine(Line)
    step = 0.005;
    points = 1/0.005;
    vec = Line.end-Line.start;
    out = zeros(3,(points+1)*2);
    out(:,1) = Line.start;
    for i = 2:points+1
        math = i*step*vec+Line.start;
        out(:,i) = math;
    end
    Line.start = Line.start - FRU2coord(0,0,35*10^-3);
    out(:,points+2) = Line.start;
    for i = points+3:(points+1)*2
        math = i*step*vec+Line.start;
        out(:,i) = math;
    end
end

