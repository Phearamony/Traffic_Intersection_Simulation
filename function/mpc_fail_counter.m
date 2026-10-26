function out = mpc_fail_counter(cmd)
%MPC_FAIL_COUNTER  Count QP solves and QP failures (fallback used) per run.
%   mpc_fail_counter('reset')   at the start of a main script
%   mpc_fail_counter('solve')   inside Car.MPC, once per solve
%   mpc_fail_counter('fail')    inside Car.MPC, when the fallback is used
%   c = mpc_fail_counter('get') returns struct with fields solves, fails
persistent n_solve n_fail
if isempty(n_solve), n_solve = 0; n_fail = 0; end
out = [];
switch cmd
    case 'reset', n_solve = 0; n_fail = 0;
    case 'solve', n_solve = n_solve + 1;
    case 'fail',  n_fail  = n_fail + 1;
    case 'get',   out = struct('solves', n_solve, 'fails', n_fail);
end
end
