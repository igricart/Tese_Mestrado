tic
% Body parameters
contact_point_h = BodyContactPositions(:,1);
contact_point_t = BodyContactPositions(:,2);

l2 = L(3,3);
l3 = L(3,4);
ah = contact_point_h(3);
aH = norm(contact_point_h);
aT = norm(contact_point_t);

% Joint reference signal
% q1 = [-0.0763 : 1e-2 : 0.0893];
q1 = [-0.0763, 0, 0.0893];
q2 = [-0.1456 : 2e-2 : 0.5886];
q3 = [-1.1060 : 1e-2 : -0.0660];
q4 = [-0.6971 : 1e-2 : -0.1317];
size_vec_rows = length(q1) * length(q2) * length(q3) * length(q4);
%% Optimization process

% Options declaration
options = optimoptions('fmincon','Algorithm','interior-point'); % run interior-point algorit

% Joint position initialization
q_init = [2;2;2;2];

%  Mapping joint angles and z coordinates from heel and toe
lookup_table = zeros(size_vec_rows, 6);

ind = 1;
for i=1:length(q1)
    for j=1:length(q2)
        for k=1:length(q3)
            for l=1:length(q4)
                q = [q1(i);q2(j);q3(k);q4(l)];

                % Direct kinematics
                z_h = q(1) + l2 * cos( q(2) ) + l3 * ( cos( q(2) + q(3) ) ) ...
                + aH * cos( q(2) + q(3) + q(4) + ( - acos( ah / aH ) ) );

                z_t = q(1) + l2 * cos( q(2) ) + l3 * ( cos( q(2) + q(3) ) ) ...
                + aT * cos( q(2) + q(3) + q(4) + ( acos( ah / aT ) ) );

                % Optimization function
                %x = fmincon(@(x) myfun(x,q),q_init,[],[],[],[],[],[],@(x) mycon(x , l2, l3, ah, aH, aT, z_h, z_t),options);

                % Use q as [cm deg deg deg] and populate the lookup_table
                lookup_table(ind, :) = [z_h z_t q(1)*100 rad2deg(q(2:4))'];
                ind = ind + 1;
            end
        end
    end
end

lookup_table = sortrows(round(1e+4 * lookup_table) / 1e+4);

% Find duplicates algorithm
z_duplicates = [];
z_ind = 1;
i = 1;

for i = 1 : size_vec_rows - 1
    % Copy to z_duplicates when z_h and z_t duplicated
    if lookup_table(i, 1 : 2) == lookup_table(i + 1, 1 : 2)
        z_duplicates(z_ind : z_ind + 1, :) = lookup_table(i : i + 1, :);
        z_ind = z_ind + 2;
    end
    if (i + 2 <= size_vec_rows)
        if lookup_table(i, 1 : 2) == lookup_table(i + 2, 1 : 2)
            z_ind = z_ind - 1;
        end
    end
end
toc