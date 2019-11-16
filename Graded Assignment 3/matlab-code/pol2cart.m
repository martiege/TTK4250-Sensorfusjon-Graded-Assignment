function c = pol2cart(p)
%POL2CART Converts polar coordinates to cartesian coordinates
%   
r   = p(1); 
phi = p(2);
x   = r * cos(phi); 
y   = r * sin(phi); 
c = [x; y]; 
end

