% Title:        AWP Trajectory Planning and Control
% File:         lspb_get.m
% Date:         2017-04-20
% Authors:      Nicolai Domingo Nielsen
%               Paul-Edouard Sarlin
% Description:  Compute LSPB trajectory references for some points in time.

function [ q, v, a ] = lspb_get( t, lspb )

% Unpack trajectory parameters
n = lspb.n;
d = lspb.d;
p = lspb.p;
T_stamps = lspb.T_stamps;
v_lin = lspb.v_lin;
a_blend = lspb.a_blend;
t_blend = lspb.t_blend;

q = zeros(length(t),d);
v = zeros(length(t),d);
a = zeros(length(t),d);

for i = 1:length(t)
    
    if t(i) < 0
        q(i,:) = p(1,:);
        continue
    elseif t(i) > T_stamps(end)
        q(i,:) = p(end,:);
        continue
    end
    
    [~,idx] = min(abs(t(i)-T_stamps));
    idx = min(idx, n);
    dt = t(i) - T_stamps(idx);
    
    if abs(dt) <= t_blend(idx)/2 
        q(i,:) = p(idx+1,:) + v_lin(idx,:)*dt ...
                            + 1/2*a_blend(idx,:)*(dt+t_blend(idx)/2).^2;
        v(i,:) = v_lin(idx,:) + a_blend(idx,:)*(dt+t_blend(idx)/2);
        a(i,:) = a_blend(idx,:);
    else
        if dt < 0
            idx = idx - 1;
            dt = t(i) - T_stamps(idx);
        end
        q(i,:) = p(idx+1,:) + v_lin(idx+1,:)*dt;
        v(i,:) = v_lin(idx+1,:);
    end
    
end


end

