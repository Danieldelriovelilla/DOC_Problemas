classdef doc_functions < handle
    properties
        
    end

    methods (Access = public)
        function obj = doc_functions()

        end

    % Cosine matrix
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
    C = C1(t1)*C2(t2)*C3(t3);
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
    
    function [C] = C_from_e(obj, phi, e)
    ex = [0, -e(3), e(2);
          e(3), 0, -e(1);
          -e(2), e(1), 0];
    C = cos(phi)*eye(3) + (1-cos(phi))*e'*e - sin(phi)*ex;
    end
    
    
    % Eigenaxis
    function [phi, e] = Eigenaxis(obj, C21)
    phi = acos( ( C21(1,1) + C21(2,2) + C21(3,3) - 1)/2 );
    e = [C21(2,3) - C21(3,2);
         C21(3,1) - C21(1,3);
         C21(1,2) - C21(2,1)]/(2*sin(phi));
    end
    
    % Quaternions
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
    
    
    % Attitude determination
    function [Cb, Ci, Cbi] = Triad_Method(obj, ub, vb, ui, vi)
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
    for i = 1:3
        signo = sign(C(1,i));
        if signo == 0
            C(:,i) = C(:,i)*1;
        else
            C(:,i) = C(:,i)*signo;
        end
    end
    % disp('C final'); disp(C)
    end

    end
end