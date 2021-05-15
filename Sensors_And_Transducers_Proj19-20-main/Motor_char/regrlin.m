% regrlin.m
% calcolo dei coefficienti di regressione lineare su due serie di dati
% legati da legge lineare
%
% uso [m, q, r] = regrlin(x, y)
% con:
%  - m = coefficiente angolare della retta di regressione
%  - q = intercetta della retta di regressione
%  - r =  coefficiente di regressione

function [m, q, r] = regrlin(x, y)

x = x(:);
y = y(:);
N = length(x);
if (N ~= length(y))
    error(sprintf('\n\nregrlin: vectors x and y must be the same length!'));
end

m = (N * sum(x.*y) - sum(x)*sum(y))/(N * sum(x.^2) - sum(x)^2);
q = (sum(y) - m*sum(x))/N;
r = abs((N * sum(x.*y) - sum(x)*sum(y))/sqrt((N*sum(x.^2) - sum(x)^2)*(N*sum(y.^2) - sum(x)^2)));
