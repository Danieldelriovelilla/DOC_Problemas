classdef doc_functions < handle
    properties
        mu = 3.986e5;   % km^3/s^2
    end

    methods (Access = public)
        function obj = doc_functions()
            disp('Thanks for using our function module!')
            disp('Go to https://github.com/Danieldelriovelilla/DOC_Problemas -> DOCUMENTATION to get more information ')
            disp(" ")
        end


    %% COSINE MATRIX

    function [C] = C_from_to(obj, from, to)
    C = zeros(3);
    for i = 1:size(from, 1)
        for j = 1:size(to, 1)
        C(i,j) = dot(from(:,i),to(:,j));
        end
    end
    disp("C = ")
    disp(C)
    end

    function [C] = C1(obj, theta)
    C = [1, 0, 0
        0, cos(theta), sin(theta);
        0, -sin(theta), cos(theta)];
    end

    function [C] = C2(obj, theta)
    C = [cos(theta), 0, -sin(theta);
        0, 1, 0
        sin(theta), 0, cos(theta)];
    end

    function [C] = C3(obj, theta)
    C = [cos(theta), sin(theta), 0;
        -sin(theta), cos(theta), 0;
        0, 0, 1];
    end

    function [C] = C323(obj, t1, t2, t3)
    C = C3(t1)*C2(t2)*C3(t3);
    end

    function [C] = C123(obj, t1, t2, t3)
    C = C1(obj, t1)*C2(obj, t2)*C3(obj, t3);
    end

    function [theta1] = theta1_321(obj, C)
    theta1 = atan2(C(2,3),C(3,3));
    end

    function [theta2] = theta2_321(obj, C)
    theta2 = -asin(C(1,3));
    end

    function [theta3] = theta3_321(obj, C)
    theta3 = atan2(C(1,2),C(1,1));
    end


    %% EIGENAXIS

    function [C] = C_from_e(obj, phi, e)
    ex = [0, -e(3), e(2);
          e(3), 0, -e(1);
          -e(2), e(1), 0];
    C = cos(phi)*eye(3) + (1-cos(phi))*e'*e - sin(phi)*ex;
    end

    function [phi, e] = Eigenaxis(obj, C21)
    phi = acos( ( C21(1,1) + C21(2,2) + C21(3,3) - 1)/2 );
    e = [C21(2,3) - C21(3,2);
         C21(3,1) - C21(1,3);
         C21(1,2) - C21(2,1)]/(2*sin(phi));
    end


    %% QUATERNIONS

    function [q] = Quaternions_from_C(obj, C)
    phi = acos( ( C(1,1) + C(2,2) + C(3,3) - 1)/2 );
    e = [C(2,3) - C(3,2);
         C(3,1) - C(1,3);
         C(1,2) - C(2,1)]/(2*sin(phi));

    q = [e(1)*sin(phi/2);
         e(2)*sin(phi/2);
         e(3)*sin(phi/2)];
    q(4) = cos(phi/2);
    end

    function [q] = Quaternions_from_e(obj, phi, e)
    q = [e(1)*sin(phi/2);
         e(2)*sin(phi/2);
         e(3)*sin(phi/2)];
    q(4) = cos(phi/2);
    end

    function [C] = C_from_quaternion(obj, q)
    q4 = q(4);
    q = [q(1);q(2);q(3)];
    C = (q4^2-q'*q)*eye(3) + 2*(q*q') - 2*q4* Skew_Sym_Mat(obj, q);
    end
    

    %% ATTITUDE DETERMINATION

    function [Cb, Ci, Cbi] = Triad_Method(obj, ub, vb, ui, vi)
    % Normalize
    ub = ub/norm(ub);    
    vb = vb/norm(vb);
    ui = ui/norm(ui);
    vi = vi/norm(vi);
    % Body
    t1b = ub;
    t2b = cross(ub, vb)/norm(cross(ub, vb));
    t3b = cross(t1b, t2b);
    % Inertial
    t1i = ui;
    t2i = cross(ui, vi)/norm(cross(ui, vi));
    t3i = cross(t1i, t2i);
    % Matrix
    Cb = [t1b, t2b, t3b];
    Ci = [t1i, t2i, t3i];
    Cbi = Cb*Ci';
    end
    
    function [q,Cbi] = q_Method(obj,VB,VI,W)
    
    % Weights 
    if ~exist('W','var')
        W = ones(size(VB,2),1);
    end
    
    % Normalize
    for k=1:size(VB,2)
        VB(:,k) = VB(:,k)/norm(VB(:,k));
        VI(:,k) = VI(:,k)/norm(VI(:,k));
    end  
         
    % K matrix
    B = zeros(3,3);
    for k=1:size(VB,2)
        B = B + W(k) * ( VB(:,k)*VI(:,k).');
    end        
    k22 = trace(B);
    K11 = B + B' - k22 * eye(3);
    k12 = [(B(2,3) - B(3,2)) ; (B(3,1) - B(1,3)) ; (B(1,2) - B(2,1))];
    K = cell2mat({K11,k12;k12',k22});
    
    % Eigenvalues
    [V,D] = eig(K);
    q = V(:,find(diag(D) == max(diag(D))));
    
    % Transformation
    Cbi = C_from_quaternion(obj,q);
    
    end
    
    function [q,Cbi] = quest_Method(obj,VB,VI,W)
    
    % Weights 
    if ~exist('W','var')
        W = ones(size(VB,2),1);
    end
    
    % Normalize
    for k=1:size(VB,2)
        VB(:,k) = VB(:,k)/norm(VB(:,k));
        VI(:,k) = VI(:,k)/norm(VI(:,k));
    end  
         
    % K matrix
    B = zeros(3,3);
    for k=1:size(VB,2)
        B = B + W(k) * ( VB(:,k)*VI(:,k).');
    end        
    lambda_max = sum(W);
    S = B + B';
    k12 = [(B(2,3) - B(3,2)) ; (B(3,1) - B(1,3)) ; (B(1,2) - B(2,1))];
    k22 = trace(B);
    
    p = inv((lambda_max + k22) * eye(3,3) - S) * k12;
    q = 1/(1+norm(p)^2)^0.5 * [p;1];
    
    % Transformation
    Cbi = C_from_quaternion(obj,q);
    
    end
    
    %% ATTITUDE DYNAMICS

    function [I, C] = Principal_Inertia(obj, J)
    % Obtain eigenvalues and eigenvectors
    [C, I] = eig(J);
    % disp('C recien calculada'); disp(C)
    % Order the inertia correcly
    [~,idx] = sort(diag(I), "descend");
    I = flip(I(:,idx));
    C = C(:,idx);
    % disp('C sort'); disp(C)
    %C = flip(C);
    % disp('C flip'); disp(C)
    % Give the corect signt to the eigenvector
%     for i = 1:3
%         signo = sign(C(1,i));
%         if signo == 0
%             C(:,i) = C(:,i)*1;
%         else
%             C(:,i) = C(:,i)*signo;
%         end
%     end
    f1 = C(:,1); f2 = C(:,2); f3 = C(:,3);
    if cross(f1,f2) ~= f3
        f3 = -f3;
    end
    C = [f1';f2';f3'];
    % Check if CJC' = I
    if round(C*J*C') == round(I)
        disp("Rotation matrix is correct.")
    else
        disp("Rotation matrix is NOT correct.")
    end
    % disp('C final'); disp(C)
    end


    function [I] = Inertia_Matrix(obj, I1, I2, I3)
    I = [I1, 0, 0;
         0, I2, 0;
         0, 0, I3];
    end

    function [I] = Inertia_Cylinder(obj, m, r, h)
    I1 = m*(3*r^2 + h^2)/12;
    I2 = I1;
    I3 = 0.5*m*r^2;
    I = Inertia_Matrix(obj, I1, I2, I3);
    end

    function [eq] = Euler_Equation(obj, I, w, dw, T)
    eq = I*dw + cross(w, I*w) == T;
    for i = 1:3
        eq(i) = isolate(eq(i), dw(i));
    end
    end

    function [T] = Kinetic_Energy(obj, I, w)
    T = 0.5 * w'*I*w;
    end

    function [h] = Angular_Momentum_Iw(obj, I, w)
    h = I*w;
    end

    function [nutation_angle] = Nutation(obj, h)
    ht = norm(h(1:2));
    hn = norm(h);
    nutation_angle = asin(ht/hn);
    end

    function [precession_rate] = Precession_Rate(obj, I, h)
    hn = norm(h);
    precession_rate = hn/I(1,1);
    end

    % Stabilization
    function [eq] = Euler_Equation_Stabilized(obj, I, w, dw, T, hs, a)
    eq = I*dw + cross(w, (I*w + hs*a)) == T;
    for i = 1:3
        eq(i) = isolate(eq(i), dw(i));
    end
    end
    
    function [Tgg] = Gravitational_Torque(obj, mu, Rob, I)
    Tgg = 3*mu*obj.Skew_Sym_Mat(Rob)*I*Rob/norm(Rob)^5;
    end
    
    function [Ax] = Skew_Sym_Mat(obj, x)
    Ax = ...
        [0, -x(3), x(2);
         x(3), 0, -x(1);
         -x(2), x(1), 0];
    end


    end
end
