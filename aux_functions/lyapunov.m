function [V1, V2] = lyapunov(param, k, ex, ev, eR, eW, psi)
    V1 = 0.5*k.x*dot(ex, ex) + 0.5*param.m*dot(ev, ev) + param.c1*dot(ex, ev);
    V2 = 0.5*dot(eW, param.J*eW) + k.R * psi + param.c2*dot(eR, eW);
end