function out = sat_func(S,omega)

    out = (sign(S)).*(abs(S)>omega) + (S./omega).*(abs(S)<=omega);

end