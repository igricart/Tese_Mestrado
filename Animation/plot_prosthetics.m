%% Plot for the prothetic leg.
protese = Prosthetics( l_base_hip, l_hip_thigh, l_thigh_shin, l_shin_foot, contact_point_h, contact_point_t, s_z );

%Adaptation
%reduction rate r
r = 100;
clear q_Red;
for i = 1:4
    q_Red(:,i) = decimate(q.Data(:,i),r);
    
end

for i = 1:size(q_Red,1)
    tic
    protese.PlotProsthetics( q_Red(i,:), L, h, contact_point_h, contact_point_t, joint_type );
    t(i) = toc;
    pause(0.00000001);
    
%     pause(q.Time(i) - q.Time(i+1))
end
