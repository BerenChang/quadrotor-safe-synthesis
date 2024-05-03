function bound = L1(V1, V2, t, param)
    bound = exp(param.alpha1*sqrt(V2)/2/param.beta)*sqrt(V1 + V2)*exp(-param.alpha0*t/2);
end
