function d = dist_point_to_line(P0, P1, P2)
%P1, P2, P0 row vectors of form [x, y]
d = abs(det([P2-P1;P0-P1]))/abs(dist(P2-P1, [0,0]));

