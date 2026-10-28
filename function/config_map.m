function [label, cfg] = config_map(name)
%CONFIG_MAP  Configuration short name -> results label (see sim/sim_config.m).
%   Names: C1, V2V_IDM, V2V_MPC, C2, C2G, C3I, C3, C4, C4_NOUNC,
%   C4_NOONLINE, C4_SPEED
cfg = sim_config(name);
label = cfg.label;
end
