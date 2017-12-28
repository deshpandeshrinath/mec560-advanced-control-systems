function [ fu ] = dampingFriction( v )
    fu = 0;
if(v > 0)
    fu = -4;
else (v < 0)
    fu =  4;
end 
end

