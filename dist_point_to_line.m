function(P0, P1, P2)
%P1, P2, P0 row vectors of form [x, y]
return abs(det([P2-P1;P0-P1]))/abs(P2-P1)
