% load('acc_state_feedback_clean.mat');
ddq_ref_res = reshape(ddq_ref.Data',4,1,size(ddq_ref.Data,1));
dq_ref_res = reshape(dq_ref.Data',4,1,size(dq_ref.Data,1));

ddq_res = reshape(ddq.Data',4,1,size(ddq.Data,1));
dq_res = reshape(dq.Data',4,1,size(dq.Data,1));

% Use acceleration and velocity from plant
ddq_ref_res = ddq_res;
dq_ref_res = dq_res;
new_q = q;
new_dq = dq;
new_ddq = ddq;

% Compute matrices and vectors
M = MassMatrix(new_q.Data, Mass, Inertia, R, L);
G = GravityVector(new_q.Data, Mass, Inertia, R, L, g);
C = CoriolisMatrix(new_q.Data, new_dq.Data, Mass, Inertia, R, L);

if(size(M,3) == size(C,3))
    for k = 1:size(M,3)
        output_M(:,:,k) = M(:,:,k)*ddq_ref_res(:,:,k);
        output_C(:,:,k) = C(:,:,k)*dq_ref_res(:,:,k);
    end
else
    for k = 1:size(M,3)
        output_M(:,k) = M(:,:,k)*ddq_ref_res(:,:,k);
    end
    for k = 1:size(C,3)
        output_C(:,k) = C(:,:,k)*dq_ref_res(:,:,k);
    end
end

output_M_res = reshape(output_M, 4, size(M,3))';
output_C_res = reshape(output_C, 4, size(C,3))';
output_G_res = reshape(G, 4, size(G,3))';

figure('Name',['Joint ' num2str(i)],'NumberTitle','off')
i = 1;
subplot(2,2,i)
plot(new_ddq.Time, output_M_res(:,i) + output_C_res(:,i) + output_G_res(:,i),'.', u.Time, u.Data(:,i));
grid on; grid minor;
ylabel(['Joint ' num2str(i) ' Force']);
xlabel(['Time (sec)']);
%legend({'Lagrange','Newton-Euler'},'Location','southeast');

i = 2;
subplot(2,2,i)
plot(new_ddq.Time, output_M_res(:,i) + output_C_res(:,i) + output_G_res(:,i),'.', u.Time, u.Data(:,i));
grid on; grid minor;
ylabel(['Joint ' num2str(i) ' Torque']);
xlabel(['Time (sec)']);
%legend({'Lagrange','Newton-Euler'},'Location','southeast');

i = 3;
subplot(2,2,i)
plot(new_ddq.Time, output_M_res(:,i) + output_C_res(:,i) + output_G_res(:,i),'.', u.Time, u.Data(:,i));
grid on; grid minor;
ylabel(['Joint ' num2str(i) ' Torque']);
xlabel(['Time (sec)']);
%legend({'Lagrange','Newton-Euler'},'Location','southeast');

i = 4;
subplot(2,2,i)
plot(new_ddq.Time, output_M_res(:,i) + output_C_res(:,i) + output_G_res(:,i),'.', u.Time, u.Data(:,i));
grid on; grid minor;
ylabel(['Joint ' num2str(i) ' Torque']);
xlabel(['Time (sec)']);
legend({'Lagrange','Newton-Euler'},'Location','southeast');

% for i=1:4
%Plot torques/forces
% figure('Name',['Joint ' num2str(i)],'NumberTitle','off')
%     % D
%     subplot(4,1,1)
%     plot(new_ddq.Time, output_M_res(:,i),'.');
%     ylabel(['M']);
%     % C
%     subplot(4,1,2)
%     plot(new_ddq.Time, output_C_res(:,i),'.');
%     ylabel(['C']);
%     % G
%     subplot(4,1,3)
%     plot(new_ddq.Time, output_G_res(:,i),'.');
%     ylabel(['G']);
%     xlabel(['Time (sec)']);
%     % All
%     subplot(4,1,4)
%     plot(new_ddq.Time, output_M_res(:,i) + output_C_res(:,i) + output_G_res(:,i),'.', u.Time, u.Data(:,i));
%     grid on; grid minor;
%     ylabel(['Joint ' num2str(i) ' Force']);
%     xlabel(['Time (sec)']);
%     legend({'Lagrange','Newton-Euler'},'Location','southeast');
%end

%% Comparison

% figure('Name','Joint 1 Force','NumberTitle','off')
% subplot(2,1,1)
% plot(ddq_ref.Time, output_M_res(:,1) + output_C_res(:,1) + output_G_res(:,1),'.', u.Time, u.Data(:,1));
% ylabel(['Joint 1 Force']);
% xlabel(['Time (sec)']);
% legend({'Lagrange','Newton-Euler'},'Location','southeast');
% 
% subplot(2,1,2)
% plot(q_ref.Time, q_ref.Data(:,1));
% ylabel(['Q1']);
% xlabel(['Time (sec)']);