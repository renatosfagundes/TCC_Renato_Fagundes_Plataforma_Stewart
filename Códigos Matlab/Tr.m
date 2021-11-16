function [M] = Tr(dx,dy,dz)
    M = [1 0 0 dx; 0 1 0 dy; 0 0 1 dz; 0 0 0 1];
end