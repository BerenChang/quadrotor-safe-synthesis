function psi_value = get_psi(R, Rd)
    psi_value = 0.5*trace(eye(3) - Rd'*R);
end