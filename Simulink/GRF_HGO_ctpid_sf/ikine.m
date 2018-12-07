
function y = ikine(input)
% global L BodyContactPositions
% 
% % Body parameters
% contact_point_h = BodyContactPositions(:,1);
% contact_point_t = BodyContactPositions(:,2);
% 
% l2 = L(3,3);
% l3 = L(3,4);
% ah = contact_point_h(3);
% aH = norm(contact_point_h);
% aT = norm(contact_point_t);

q_leg = input(1:4);
z = input(5:6);

l2 = 0.5;
l3 = 0.4;
ah = 0.1;
aH = 0.1414;
aT = 0.2236;
 
%% Optimization process

% Options declaration
% options = optimoptions('fmincon','Algorithm','interior-point'); % run interior-point algorit

% Joint position initialization
q_init = [2;2;2;2];

z_h = z(1);
z_t = z(2);

% Optimization functin
x = fmincon(@(x) myfun(x,q_leg),q_init,[],[],[],[],[],[],@(x) mycon(x , l2, l3, ah, aH, aT, z_h, z_t))
y = x;
end