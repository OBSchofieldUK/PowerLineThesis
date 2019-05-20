function line3d = LineTo3D(Line)
    line3d = [Line.start;Line.end-Line.start];
    line3d = Normalize3Dline(line3d);
end