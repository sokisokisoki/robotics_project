function [q_lambda,lambda,P0T_lambda,R0T_lambda] = qpPathGen_positionOnly(robot,q0,P0Td,...
    epsilon_p,q_prime_min,q_prime_max,N)

    n = length(q0);
    lambda = 0:1/(N):1;
    options = optimoptions('quadprog','Display','off');

    [R0T0, P0T0] = fwdkin(robot,q0); 

    % Compute Path in Task Space
    Pdes_lambda = zeros(3,length(lambda));
    dP0T_dlambda = (P0Td - P0T0); % constant in lambda
    for k = 1:length(lambda)
         Pdes_lambda(:,k) = (1-lambda(k))*P0T0 + lambda(k)*P0Td;
    end
    % Solve QP Problem and Generate Joint Space Path
    q_prime = zeros(n,length(lambda)); q_lambda = zeros(n,length(lambda)); q_lambda(:,1) = q0;
    exitflag = zeros(1,length(lambda)); P0T_lambda = zeros(3,length(lambda));
    R0T_lambda = zeros(3,3,length(lambda)); P0T_lambda(:,1) = P0T0; R0T_lambda(:,:,1) = R0T0;
    qprev = q0; 
    for k = 1:length(lambda)
        [lb,ub] = qprimelimits_full(robot.qlimit,qprev,N,q_prime_max,q_prime_min);
        J = robotjacobian(robot, qprev);
        %%%%%%%%%%%%%%%%%%%%%%%%
        %Hint: See line 12
        vt = dP0T_dlambda;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %Hint: We only need the position rows of the Jacobian. MATLAB index starts at 1.
        %To get rows i through j (including j) we use J(i:j,:)
        H = getqp_H_positionOnly(qprev, J(4:6,:), vt, epsilon_p);
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        f = getqp_f_positionOnly( qprev, epsilon_p );
        [q_prime_temp,~,exitflag(k)] = quadprog(H,f,[],[],[],[],lb,ub,[],options);
        q_prime_temp = q_prime_temp(1:n);
        % check exit flag - all elements should be 1
        if exitflag(k) ~= 1
            disp('Generation Error')
            return;
        end
        q_prime(:,k) = q_prime_temp; qprev = qprev + (1/N)*q_prime_temp;
        q_lambda(:,k+1) = qprev;   
        [Rtemp, Ptemp] = fwdkin(robot,qprev);
        P0T_lambda(:,k+1) = Ptemp;  R0T_lambda(:,:,k+1) = Rtemp;
    end

    % Chop off excess
    q_lambda(:,end) = []; P0T_lambda(:,end) = []; R0T_lambda(:,:,end) = [];
end

function [lb,ub] = qprimelimits_full(qlimit,qprev,N,qpmax,qpmin)
    n = length(qlimit);
    % Compute limits due to joint stops
    lb_js = N*(qlimit(:,1) - qprev);
    ub_js = N*(qlimit(:,2) - qprev);
    % Compare and find most restrictive bound
    lb = zeros(n+2,1); ub = zeros(n+2,1); ub(end-1) = 1; ub(end) = 1;
    for k = 1:n
        if lb_js(k) > qpmin(k)
            lb(k) = lb_js(k);
        else
            lb(k) = qpmin(k);
        end   
        if ub_js(k) < qpmax(k)
            ub(k) = ub_js(k);
        else
            ub(k) = qpmax(k);
        end
    end
end

function [ H ] = getqp_H_positionOnly(dq, J, vp, ep)
    n = length(dq);
    H1 = [J,zeros(3,1)]'*[J,zeros(3,1)];
    
    H2 = [zeros(3,n),zeros(3,1);zeros(3,n),vp]'*[zeros(3,n),zeros(3,1);zeros(3,n),vp];
    
    H3 = -2*[J,zeros(3,1)]'*[zeros(3,n),vp];
    H3 = (H3+H3')/2;
    H4 = [zeros(1,n),0;zeros(1,n),sqrt(ep)]'*[zeros(1,n),0;zeros(1,n),sqrt(ep)];
    H = 2*(H1+H2+H3+H4);
end


function [ f ] = getqp_f_positionOnly( dq, ep )
    f = -2*[zeros(1,length(dq)),ep]';
end