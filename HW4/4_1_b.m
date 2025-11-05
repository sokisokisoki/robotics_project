
function [q,p0T,RPY0T,iternum]=ik_Jtranspose(robot,q0,Rd,Pd,Nmax,alpha,tol)
   
% set up storage space
    n=length(q0);    
    q=zeros(n,Nmax+1);
    q(:,1)=q0; % output joint displacements
    p0T=zeros(3,Nmax+1); % output p
    RPY0T=zeros(3,Nmax+1); % output Euler angles
    iternum = 1;
    [R,P] = fwdkin(robot,q0);
    dR = R*Rd';
   %get the pose error
    dX = [flip(rotm2eul(dR))';P-Pd];
    % iterative update
    while ~prod((abs(dX))<tol)
        if iternum <= Nmax
            % forward kinematics
            [R,p0T(:,iternum)]=fwdkin(robot,q(:,iternum));
            Jq = robotjacobian(robot,q(:,iternum));
            RPY0T(:,iternum)=flip(rotm2eul(R));
          %get the pose error
            dR = R*Rd';
            dX = [flip(rotm2eul(dR))';p0T(:,iternum)-Pd];
            % Jacobian update       
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %dX = [beta s; P-Pd]
            q(:,iternum+1)=q(:,iternum)-alpha*Jq'*dX;  
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            iternum = iternum+1;
        else
            break;
        end
    end
    q(:,iternum:end) = [];
    p0T(:,iternum:end) = [];
    RPY0T(:,iternum:end) = [];
    iternum = iternum-1;
end


