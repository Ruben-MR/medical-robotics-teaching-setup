%#codegen
function [ pinvA ] = weightedPseudoInverseMat(A, lambda, invWeight)
% Input: Any m-by-n matrix.
% Output: An  n-by-m pseudo-inverse of the input according to the Moore-Penrose damped version formula

% Get the number of rows (m) and columns (n) of A
[m,n] = size(A);

if (m>n)
  % Compute the left pseudoinverse.
  pinvA = (A'*A - lambda*lambda*eye(n,n))\A';
elseif (m<=n)
  % Compute the right pseudoinverse.
    pinvA = invWeight*A'/(A*invWeight*A' - lambda*lambda*eye(m,m));
end
end
