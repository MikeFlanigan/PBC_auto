clc
heading = 0;
wp = 335;

diff = heading - wp
if diff > 0
    if diff < 180
        disp('turn left')
    else
        disp('turn right')
        180-mod(diff,180)
    end
end
if diff < 0 
    if diff > -180
        disp('turn right')
    else
        disp('turn left')
        mod(diff,180)
    end
end
    
