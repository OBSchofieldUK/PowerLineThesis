function printVector(vector,name)
    text = string(name) + ' [' + string(vector(1));
    for i = 2:length(vector)
       text = text +', '+ string(vector(i)); 
    end
    text = text +']';
    disp(text)
end