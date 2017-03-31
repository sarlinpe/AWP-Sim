clear; close all;

% Initial constraints
%p = [0,1;5,6;3,3;7,8;1,2];
p = [0,0;1,2;3,3;5,2.5;6,4];
v_max = [1,1];
a_max = [0.6,0.6];

% Initialise parameters
[n,d] = size(p);
t_blend = zeros(1,n);
T = zeros(1,n-1);
v_lin = zeros(n+1,d);
a_blend = zeros(n,d);

% Compute duration and velocity of linear segments
for i = 1:n-1
    T(i) = max( abs(p(i+1,:) - p(i,:))./v_max );
end
T = [1 T 1]; % dummy, will be replaced later
p = [p(1,:); p; p(end,:)]; % duplicate ends
for i = 1:n+1
    v_lin(i,:) = (p(i+1,:) - p(i,:))/T(i);
end

% Compute duration and acceleration of parabolic blends
for i = 1:n
    t_blend(i) = max( abs(v_lin(i+1,:) - v_lin(i,:))./a_max );
    a_blend(i,:) = (v_lin(i+1,:) - v_lin(i,:))/t_blend(i);
end
T(1) = t_blend(1)/2;
T(end) = t_blend(end)/2;
T_stamps = cumsum(T);

% A small test to evalutate the path in time
t = linspace(0,sum(T));
q = zeros(length(t),d);

for i = 1:length(t)
    
    [~,idx] = min(abs(t(i)-T_stamps));
    idx = min(idx, n);
    dt = t(i) - T_stamps(idx);
    
    if abs(dt) <= t_blend(idx)/2 
        q(i,:) = p(idx+1,:) + v_lin(idx,:)*dt + 1/2*a_blend(idx,:)*(dt+t_blend(idx)/2).^2;
    else
        if dt < 0
            idx = idx - 1;
            dt = t(i) - T_stamps(idx);
        end
        q(i,:) = p(idx+1,:) + v_lin(idx+1,:)*dt;
    end
    
end

figure()
plot(t,q(:,1),'LineWidth',2)
line([0 T_stamps],p(:,1),'Color','r','LineStyle','--')

figure()
plot(q(:,1),q(:,2),'LineWidth',2)
line(p(:,1),p(:,2),'Color','r','LineStyle','--')