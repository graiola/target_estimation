function [x, J_history] = gradientDescent(J, x, alpha, max_num_iters)

num_iters = 0;
J_curr = J(x);
J_history = zeros(length(x),max_num_iters);

while abs(J_curr) > 0.00001 && num_iters < max_num_iters

    x = x - alpha .* J_curr; 
    
    % Update for the next iteration
    J_curr = J(x);
    num_iters = num_iters + 1;
        
        
    % Save the cost J in every iteration    
    J_history(num_iters) = J_curr;

end
