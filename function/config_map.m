function [label, cfg] = config_map(name)
%CONFIG_MAP  Configuration short name -> results label (see sim/sim_config.m).
%   Names: C1, V2V_IDM, V2V_MPC, C2, C2G, C3I, C3, C4, C4_NOUNC,
%   C4_NOONLINE, C4_SPEED, C4_STRICT, C4_SEQ, C4_NOADV, C4_ONCE, C4_NOYIELD
cfg = sim_config(name);
label = cfg.label;
end
