function [x, ok] = qp_solve(H, f, A, b, lb, ub, x0)
%QP_SOLVE  min 0.5 x'Hx + f'x  s.t.  A x <= b,  lb <= x <= ub.
%   MATLAB: quadprog (Optimization Toolbox). Octave (used for testing):
%   the built-in qp. ok = false when the solver reports no solution.
persistent useQuadprog opts
if isempty(useQuadprog)
    useQuadprog = exist('quadprog', 'file') == 2 && exist('OCTAVE_VERSION', 'builtin') == 0;
    if useQuadprog
        opts = optimoptions('quadprog', 'Display', 'off', 'MaxIterations', 200, ...
                            'OptimalityTolerance', 1e-7, 'ConstraintTolerance', 1e-7);
    end
end
H = 0.5 * (H + H');
if useQuadprog
    [x, ~, flag] = quadprog(H, f, A, b, [], [], lb, ub, [], opts);
    ok = flag > 0 && ~isempty(x) && all(isfinite(x));
else
    [x, ~, info] = qp(x0, H, f, [], [], lb, ub, [], A, b);
    ok = info.info == 0 && all(isfinite(x));
    if ok && ~isempty(A)
        ok = all(A * x <= b + 1e-5);
    end
end
if ~ok, x = x0; end
end
