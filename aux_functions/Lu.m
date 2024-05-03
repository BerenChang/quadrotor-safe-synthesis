function bound = Lu(V1, V2, tm, param)
    bound = L1(V1, V2, 0, param) + L2(V2, tm, param);
end
