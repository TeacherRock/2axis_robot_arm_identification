function StrDisp(str, varargin)
res = '>> -----------------------------------------------------------------------';
if ~isempty(varargin), res = strrep(res, '-', varargin{1}); end

res(4 : 3 + length(str) + 1) = [str, ' '];

disp(res);
end