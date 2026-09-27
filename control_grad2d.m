function [ ux, uy ] = control_grad2d( p_sen, p_tar, R, J)
% Information-rate gradient component; DOI: 10.1016/j.inffus.2018.01.010.
% See docs/algorithm.md, docs/running.md, and README.md#citation.

    % ENU frame
    x = p_sen(1) - p_tar(1);
    y = p_sen(2) - p_tar(2);
    z = p_sen(3) - p_tar(3);

    H = [y/(x^2+y^2), -x/(x^2+y^2);
      -x*z/((x^2+y^2+z^2)*sqrt(x^2+y^2)), -z*y/((x^2+y^2+z^2)*sqrt(x^2+y^2));
      x/sqrt(x^2+y^2+z^2), y/sqrt(x^2+y^2+z^2)];
    
    dHdx = [-y*(2*x)/(x^2+y^2)^2, -1/(x^2+y^2) + 2*x^2/(x^2+y^2)^2;
        -z/((x^2+y^2+z^2)*sqrt(x^2+y^2)) + x*z/((x^2+y^2+z^2)*sqrt(x^2+y^2))^2 * (2*x*sqrt(x^2+y^2)+(x^2+y^2+z^2)*(x^2+y^2)^(-1/2)*x), ...
        y*z/((x^2+y^2+z^2)*sqrt(x^2+y^2))^2 * (2*x*sqrt(x^2+y^2)+(x^2+y^2+z^2)*(x^2+y^2)^(-1/2)*x);
        (x^2+y^2+z^2)^(-1/2) - x^2*(x^2+y^2+z^2)^(-3/2), -y*x*(x^2+y^2+z^2)^(-3/2)];
    
    dHdy = [1/(x^2+y^2) - 2*y^2/(x^2+y^2)^2, 2*x*y/(x^2+y^2)^2;
        x*z/((x^2+y^2+z^2)*sqrt(x^2+y^2))^2 * (2*y*sqrt(x^2+y^2)+(x^2+y^2+z^2)*(x^2+y^2)^(-1/2)*y), ...
        -z/((x^2+y^2+z^2)*sqrt(x^2+y^2)) + z*y/((x^2+y^2+z^2)*sqrt(x^2+y^2))^2 * (2*y*sqrt(x^2+y^2)+(x^2+y^2+z^2)*(x^2+y^2)^(-1/2)*y);
        -x*y*(x^2+y^2+z^2)^(-3/2), (x^2+y^2+z^2)^(-1/2) - y^2*(x^2+y^2+z^2)^(-3/2)];
    
    dPSIdx = dHdx'*(R\H) + H'*(R\dHdx);
    dPSIdy = dHdy'*(R\H) + H'*(R\dHdy);

    grad_c_x = -0.5*trace(J(1:2,1:2)\dPSIdx);
    grad_c_y = -0.5*trace(J(1:2,1:2)\dPSIdy);
   
    ux = grad_c_x;
    uy = grad_c_y;

end

