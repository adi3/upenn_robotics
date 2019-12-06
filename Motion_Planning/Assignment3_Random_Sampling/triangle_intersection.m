function flag = triangle_intersection(P1, P2)
% triangle_test : returns true if the triangles overlap and false otherwise

%%% All of your code should be between the two lines of stars.
% *******************************************************************

flag = check(P1, P2) && check(P2, P1);   
    
function bool = check(t1, t2)
    for i = 1:3
        j = mod(i, 3) + 1;
        k = mod(j, 3) + 1;
        pt1 = t1(i, :);
        pt2 = t1(j, :);
        pt3 = t1(k, :);
        d1 = 0;
        d2 = 0;
        
        if pt1(1) == pt2(1)
            % Case: Vertical line
            d1 = pt3(1) - pt1(1);
            d2 = t2(:,1) - pt1(1);
        else
            % Case: Straight line w finite slope
            m = (pt2(2) - pt1(2))/(pt2(1) - pt1(1));
            c = pt2(2) - m * pt2(1);
            d1 = pt3(2) - (m*pt3(1) + c);
            d2 = t2(:,2) - (m.*t2(:,1) + c);
        end
        
        diff_side = (d1 * d2) < 0;
        if all(diff_side)   % separating line found
            bool = false;
            return;
        end
    end
    bool = true;
end

% *******************************************************************
end