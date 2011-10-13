function d = dist_point_to_line(P0, P1, P2)
%P1, P2, P0 row vectors of form [x, y]
% d = abs(det([P2-P1;P0-P1]))/abs(dist(P2,P1));


d = abs((P2(1)-P1(1))*(P1(2)-P0(2)) - (P1(1)-P0(1))*(P2(2)-P1(2)));
d = d / sqrt((P2(1)-P1(1))^2 + (P2(2)-P1(2))^2);