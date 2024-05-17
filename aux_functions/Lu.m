function bound = Lu(V1, V2, param)
    a0 = param.alpha0;
    a1 = param.alpha1;
    a2 = param.alpha2;
    b = param.beta;

    tm = 0;
    if (a0/2 ~= b) && (V2 > 0)
        tm = max(0, 1/(a0/2 - b)*log((a2*b*sqrt(V2)/(2*b-a0))/(a0/2*sqrt(V1+V2) + a0*a2*sqrt(V2)/2/(2*b-a0))));
    elseif (a0/2 == b) && (V2 > 0)
        tm = max(0, 2*(a2*sqrt(V2)-a0*sqrt(V1+V2)/a0/a2/sqrt(V2)));
    end

    L1 = exp(a1*sqrt(V2)/2/b)*sqrt(V1 + V2)*exp(-a0*tm/2);
    L2 = exp(a1*sqrt(V2)/2/b)*(a2*sqrt(V2)/2)*exp(-a0/2*tm) ...
        *(exp((a0/2 - b)*tm) - 1) / (a0/2 - b);
    
    bound = L1 + L2;
end
