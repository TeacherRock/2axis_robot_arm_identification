function [Beta, par] = Parameter(~)

par.ax = 2;
sz = [par.ax, 1];

%% ALLParameter
SYM = {'ZZR1', 'Lzz2', 'Ix2', 'Iy2', 'Ia2', 'fv1', 'fv2', 'fs1', 'fs2'};

Beta = [0.5184    0.0844    0.3214    0.1782    0.0516  ...
        2.0200    0.6572    0.9886    0.8326];

info = {'SYM', 'Beta'};
for i = 1 : length(info)
    par.(info{i}) = eval(info{i});
end

end

