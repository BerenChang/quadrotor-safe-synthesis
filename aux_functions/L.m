function bound = L(V1, V2, t, param)
    bound = L1(V1, V2, t, param) + L2(V2, t, param);
end
