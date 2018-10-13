% This script sets up and saves the state space model for the LQR controller

clear;clear all;clc;

[df_dstate,df_dstate_sym,df_dcontrol,G,force_to_thrusteffort_forward,force_to_thrusteffort_backward,thrust_allocation] = lqr_config();

% Save parameters for matlab controller
try
    save('~/au_everything/catkin_ws/src/au_control/src/matlab_model/LQR_params.mat','df_dstate','df_dcontrol','G','force_to_thrusteffort_forward','force_to_thrusteffort_backward','thrust_allocation')
    create_path = 0;
catch
    fprintf('Please verify the correct path up to but not including /au_everything, then press the ENTER key to continue\n')
    ARVP_HOME = uigetdir('~','au_everything');
    ARVP_path = strcat(ARVP_HOME,'/au_everything/catkin_ws/src/au_control/src/matlab_model/LQR_params.mat');
    save(ARVP_path,'df_dstate','df_dcontrol','G','force_to_thrusteffort_forward','force_to_thrusteffort_backward','thrust_allocation')
    create_path = 1;
end
    
% Save parameters for python controller:

    % Convert matlab function handles to c code
    A_matrix = strrep(ccode(df_dstate_sym),'df_dstate_sym','df_dstate');
    A_matrix = strrep(A_matrix,';','');
    G_matrix = strrep(ccode(sym(G)),'T','G');
    G_matrix = strrep(G_matrix,';','');
    B_matrix = strrep(ccode(sym(df_dcontrol)),'T','B');
    B_matrix = strrep(B_matrix,';','');
    thrust_allocation_matrix = strrep(ccode(sym(thrust_allocation)),'T','thrust_allocation');
    thrust_allocation_matrix = strrep(thrust_allocation_matrix,';','');

    % get path
    if create_path == 1
        ARVP_path = strcat(ARVP_HOME,'/au_everything/catkin_ws/src/au_control/params');
        cd ARVP_path
    else
        cd '~/au_everything/catkin_ws/src/au_control/params'
    end

    % generate and save .txt file
    file_id = fopen('lqr_params_export_python.txt','w');
    fprintf(file_id,'# Params for lqr_params.py:\n\n%s\n\n%s\n\n# Params for A1 function:\n\n%s\n\n# Params for G function:\n\n%s\n\n',B_matrix,thrust_allocation_matrix,A_matrix,G_matrix);
    fclose(file_id);

fprintf('the model has been updated.\n')


% Save parameters for c++ controller:

    % Convert matlab function handles to c code
    A_matrix = strrep(ccode(df_dstate_sym),'df_dstate_sym','df_dstate');
    G_matrix = strrep(ccode(sym(G)),'T','G');
    B_matrix = strrep(ccode(sym(df_dcontrol)),'T','B');
    thrust_allocation_matrix = strrep(ccode(sym(thrust_allocation)),'T','thrust_allocation');

    % get path
    if create_path == 1
        ARVP_path = strcat(ARVP_HOME,'/au_everything/catkin_ws/src/au_control/params');
        cd ARVP_path
    else
        cd '~/au_everything/catkin_ws/src/au_control/params'
    end

    % generate and save .txt file
    file_id = fopen('lqr_params_export.txt','w');
    fprintf(file_id,'/auri/lqr_params/A_matrix:\n%s\n\n/auri/lqr_params/B_matrix:\n%s\n\n/auri/lqr_params/G_matrix:\n%s\n\n/auri/lqr_params/thrust_allocation:\n%s\n\n',A_matrix,B_matrix,G_matrix,thrust_allocation_matrix);
    fclose(file_id); 
