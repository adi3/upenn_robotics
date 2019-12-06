function X = Nonlinear_Triangulation(K, C1, R1, C2, R2, C3, R3, x1, x2, x3, X0)
%% Nonlinear_Triangulation
% Refining the poses of the cameras to get a better estimate of the points
% 3D position
% Inputs: 
%     K - size (3 x 3) camera calibration (intrinsics) matrix
%     x
% Outputs: 
%     X - size (N x 3) matrix of refined point 3D locations 

n = size(X0, 1);
X = zeros(n, 3);

for i = 1:n
    X(i,:) = Single_Point_Nonlinear_Triangulation(K, C1, R1, C2, R2, C3, R3, x1(i,:), x2(i,:), x3(i,:), X0(i,:));
end

end

function X = Single_Point_Nonlinear_Triangulation(K, C1, R1, C2, R2, C3, R3, x1, x2, x3, X0)
    n_iter = 3;
    X = X0';
    for i = 1:n_iter
        J1 = Jacobian_Triangulation(C1, R1, K, X0);
        J2 = Jacobian_Triangulation(C2, R2, K, X0);
        J3 = Jacobian_Triangulation(C3, R3, K, X0);
        J = [J1; J2; J3];
        
        f1 = K*R1*(X0' - C1);
        f2 = K*R2*(X0' - C2);
        f3 = K*R3*(X0' - C3);
        f = [f1(1)/f1(3) f1(2)/f1(3) f2(1)/f2(3) f2(2)/f2(3) f3(1)/f3(3) f3(2)/f3(3)]';
        
        b = [x1 x2 x3]';
        d_x = (J'*J)\J'*(b-f);  % A\b ~ inv(A)*b
        X = X0' + d_x;
    end
end

function J = Jacobian_Triangulation(C, R, K, X)
    f = K(1,1);
    px = K(1,3);
    py = K(2,3);
    
    x = K*R*(X'-C);
    [u,v,w] = deal(x(1), x(2), x(3));
    
    du_dx = [f*R(1,1)+px*R(3,1) f*R(1,2)+px*R(3,2) f*R(1,3)+px*R(3,3)];
    dv_dx = [f*R(2,1)+py*R(3,1) f*R(2,2)+py*R(3,2) f*R(2,3)+py*R(3,3)];
    dw_dx = [R(3,1) R(3,2) R(3,3)];
    
    df_dx = [(w.*du_dx - u.*dw_dx)./w^2; (w.*dv_dx - v.*dw_dx)./w^2];
    J = df_dx;
end
