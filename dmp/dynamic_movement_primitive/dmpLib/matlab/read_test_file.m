function read_test_file(fname, demo_fname)

clear all; close all;

fname = '../test/result/0_test_dmp_trajectory_icra2009_debug.clmc';
demo_fname = '../test/data/test_dmp_trajectory.clmc';

[D,vars,freq] = clmcplot_convert(fname);
[demo_D,demo_vars,demo_freq] = clmcplot_convert(demo_fname);

if(length(demo_D) > length(D))
    d = length(demo_D) - length(D);
    if(mod(d,2))
       error('Length of the two trajectories is wrong');
    end
    c = d/2;
    demo_D = demo_D(c+1:length(demo_D)-c, :);
end

quat_names = cell(4,1);
for i=1:4
    quat_names{i} = sprintf('TR_HAND_des_q%i_tar_x', i-1);  
end
angular_vel_names = cell(3,1);
for i=1:3
    angular_vel_names{i} = sprintf('TR_HAND_des_q%i_tar_xd', i);  
end
angular_acc_names = cell(3,1);
for i=1:3
    angular_acc_names{i} = sprintf('TR_HAND_des_q%i_tar_xdd', i);  
end

demo_quat_names = cell(4,1);
for i=1:4
    demo_quat_names{i} = sprintf('R_HAND_q%i', i-1);  
end
demo_angular_vel_names = cell(3,1);
demo_angular_vel_names{1} = 'R_HAND_ad'; 
demo_angular_vel_names{2} = 'R_HAND_bd'; 
demo_angular_vel_names{3} = 'R_HAND_gd'; 
demo_angular_acc_names = cell(3,1);
demo_angular_acc_names{1} = 'R_HAND_add';  
demo_angular_acc_names{2} = 'R_HAND_bdd';  
demo_angular_acc_names{3} = 'R_HAND_gdd';  

quat_index = zeros(4,1);
angular_vel_index = zeros(3,1);
angular_acc_index = zeros(3,1);
for i=1:length(vars)
    for j=1:4
        if(strcmp(vars(i).name, quat_names{j}))
            quat_index(j) = i;
        end
    end
    for j=1:3
        if(strcmp(vars(i).name, angular_vel_names{j}))
            angular_vel_index(j) = i;
        end
    end
    for j=1:3
        if(strcmp(vars(i).name, angular_acc_names{j}))
            angular_acc_index(j) = i;
        end
    end
end

if(find(quat_index == 0))
    error('Could not find Quaternion');
end
if(find(angular_vel_index == 0))
    error('Could not find angular velocity');
end
if(find(angular_acc_index == 0))
    error('Could not find angular acceleration');
end

quats = [D(:,quat_index(1)), D(:,quat_index(2)), D(:,quat_index(3)), D(:,quat_index(4))];
angular_vel = [D(:,angular_vel_index(1)), D(:,angular_vel_index(2)), D(:,angular_vel_index(3))];
angular_acc = [D(:,angular_acc_index(1)), D(:,angular_acc_index(2)), D(:,angular_acc_index(3))];

quat_norm = zeros(length(quats), 1);
for i=1:length(quats)
   quat_norm(i) = norm(quats(i,:));
    if(abs(quat_norm(i) - 1.0) > 0.00001)
        fprintf('Quaternion at %i is not normalized >%f %f %f %f<', i,quats(i,:));
        error();
    end
end

demo_quat_index = zeros(4,1);
demo_angular_vel_index = zeros(3,1);
demo_angular_acc_index = zeros(3,1);
for i=1:length(demo_vars)
    for j=1:4
        if(strcmp(demo_vars(i).name, demo_quat_names{j}))
            demo_quat_index(j) = i;
        end
    end
    for j=1:3
        if(strcmp(demo_vars(i).name, demo_angular_vel_names{j}))
            demo_angular_vel_index(j) = i;
        end
    end
    for j=1:3
        if(strcmp(demo_vars(i).name, demo_angular_acc_names{j}))
            demo_angular_acc_index(j) = i;
        end
    end
end

if(find(demo_quat_index == 0))
    error('Could not find Quaternion');
end
if(find(demo_angular_vel_index == 0))
    error('Could not find angular velocity');
end
if(find(demo_angular_acc_index == 0))
    error('Could not find angular acceleration');
end

demo_quats = [demo_D(:,demo_quat_index(1)), demo_D(:,demo_quat_index(2)), demo_D(:,demo_quat_index(3)), demo_D(:,demo_quat_index(4))];
demo_angular_vel = [demo_D(:,demo_angular_vel_index(1)), demo_D(:,demo_angular_vel_index(2)), demo_D(:,demo_angular_vel_index(3))];
demo_angular_acc = [demo_D(:,demo_angular_acc_index(1)), demo_D(:,demo_angular_acc_index(2)), demo_D(:,demo_angular_acc_index(3))];


size(quats)
size(demo_quats)
keyboard

quat_error = sum((quats-demo_quats).^2);
fprintf('quat error: %f %f %f %f\n', quat_error);
angular_vel_error = sum((angular_vel-demo_angular_vel).^2);
fprintf('angular vel error: %f %f %f\n', angular_vel_error);
angular_acc_error = sum((angular_acc-demo_angular_acc).^2);
fprintf('angular acc error: %f %f %f\n', angular_acc_error);

figure(1)
hold on;
for i=1:4
    subplot(4,3,(i-1)*3+1)
    plot(quats(:,i), 'g');
    title(sprintf(regexprep(quat_names{i},'_',' ')));
    subplot(4,3,(i-1)*3+2)
    plot(demo_quats(:,i), 'b');
    title(sprintf(regexprep(demo_quat_names{i},'_',' ')));
    subplot(4,3,(i-1)*3+3)
    hold on;
    title(sprintf('error: %f', quat_error(i)));
    plot(quats(:,i), 'g');
    plot(demo_quats(:,i), 'b');
    box on;
    hold off;
end
box on;
hold off;

figure(2)
hold on;
for i=1:3
    subplot(3,3,(i-1)*3+1)
    plot(angular_vel(:,i), 'g');
    title(sprintf(regexprep(angular_vel_names{i},'_',' ')));
    subplot(3,3,(i-1)*3+2)
    plot(demo_angular_vel(:,i), 'b');
    title(sprintf(regexprep(demo_angular_vel_names{i},'_',' ')));

    subplot(3,3,(i-1)*3+3)
    hold on;
    plot(angular_vel(:,i), 'g');
    plot(demo_angular_vel(:,i), 'b');
    title(sprintf('error: %f', angular_vel_error(i)));
    box on;
    hold off;
end
box on;
hold off;

figure(3)
hold on;
for i=1:3
    subplot(3,3,(i-1)*3+1)
    plot(angular_acc(:,i), 'g');
    title(sprintf(regexprep(angular_acc_names{i},'_',' ')));
    subplot(3,3,(i-1)*3+2)
    plot(demo_angular_acc(:,i), 'b');
    title(sprintf(regexprep(demo_angular_acc_names{i},'_',' ')));

    subplot(3,3,(i-1)*3+3)
    hold on;
    title(sprintf('error: %f', angular_acc_error(i)));
    plot(angular_acc(:,i), 'g');
    plot(demo_angular_acc(:,i), 'b');
    box on;
    hold off;
end
box on;
hold off;

