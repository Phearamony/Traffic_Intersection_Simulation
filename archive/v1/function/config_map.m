function [script, label] = config_map(name)
%CONFIG_MAP  Configuration short name -> main script and results label.
switch name
    case 'C1',      script = 'main_human_driving_plot'; label = 'C1_human';
    case 'V2V_IDM', script = 'main_v2v_idm';            label = 'V2V_IDM';
    case 'V2V_MPC', script = 'main_v2v_mpc';            label = 'V2V_MPC';
    case 'C2',      script = 'main_v2v_idm_rsu';        label = 'C2_idm_rsu';
    case 'C2G',     script = 'main_v2v_idm_rsu_gpr';    label = 'C2G_idm_rsu_gpr';
    case 'C3I',     script = 'main_v2v_mpc_rsu';        label = 'C3I_mpc_rsu';
    case 'C3',      script = 'main_v2v_mpc_rsu_gpr';    label = 'C3_mpc_rsu_gpr';
    otherwise, error('config_map: unknown configuration %s', name);
end
end
