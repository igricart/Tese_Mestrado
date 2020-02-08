load('acc_state_feedback_clean.mat');
ddq_ref_res = reshape(ddq_ref.Data,4,1,size(ddq_ref.Data,1));
dq_ref_res = reshape(dq_ref.Data,4,1,size(dq_ref.Data,1));
M = MassMatrix(q_ref.Data, Mass, Inertia, R, L);
G = GravityVector(q_ref.Data, Mass, Inertia, R, L, g);
C = CoriolisMatrix(q_ref.Data, dq_ref.Data, Mass, Inertia, R, L);

if(size(M,3) == size(C,3))
    for k = 1:size(M,3)
        output_M(:,:,k) = M(:,:,k)*ddq_ref_res(:,:,k);
        output_C(:,:,k) = C(:,:,k)*dq_ref_res(:,:,k);
    end
else
    for k = 1:size(M,3)
        output_M(:,:,k) = M(:,:,k)*ddq_ref_res(:,:,k);
    end
    for k = 1:size(C,3)
        output_C(:,:,k) = C(:,:,k)*dq_ref_res(:,:,k);
    end
end
output_M_res = reshape(output_M,size(M,3),4);
figure();
plot(output_M_res,'.');
output_C_res = reshape(output_C,size(C,3),4);
figure();
plot(output_C_res,'.');
output_G_res = reshape(G,size(G,3),4);
figure();
plot(output_G_res,'.');

