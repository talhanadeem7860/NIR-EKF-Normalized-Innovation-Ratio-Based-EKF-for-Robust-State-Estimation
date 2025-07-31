function D = compute_diff(y_curr,y_prev,x_curr,x_prev)

D = [(y_curr(1) - y_prev(1))/(x_curr(1) - x_prev(1)) (y_curr(1) - y_prev(1))/(x_curr(2) - x_prev(2)) (y_curr(1) - y_prev(1))/(x_curr(3) - x_prev(3)) (y_curr(1) - y_prev(1))/(x_curr(4) - x_prev(4)) (y_curr(1) - y_prev(1))/(x_curr(5) - x_prev(5));...
     (y_curr(2) - y_prev(2))/(x_curr(1) - x_prev(1)) (y_curr(2) - y_prev(2))/(x_curr(2) - x_prev(2)) (y_curr(2) - y_prev(2))/(x_curr(3) - x_prev(3)) (y_curr(2) - y_prev(2))/(x_curr(4) - x_prev(4)) (y_curr(2) - y_prev(2))/(x_curr(5) - x_prev(5))]';
end
