qstart = [90; 90; 90; 90; 90]; %initial joint configuration in degrees
qend = [30; 45; 60; 15; 90]; %final joint configuration in degrees

N = 100; %the number of sample points along the path
lambda = 0:1/(N-1):1; %the path variable -> 0 is start, 1 is end

%pre-allocate space for variables
q = zeros(5,N); %q(lambda)
Rot = zeros(3,3,N);  %Rot(lambda) (N is number of points along path. For each point, Rot is a 3x3 matrix. zeros(rows,columns,depth))
eulerot = zeros(3,N);   %Rot as Euler angles (For each path point, there will be three Euler angles)
Pot = zeros(3,N);  %Pot(lambda)

for ii = 1:N
    q(:,ii) = qstart*(1-lambda(ii)) + qend*lambda(ii); %create q(lambda)
    [Rot(:,:,ii),Pot(:,ii)] = fwdkin_Dofbot(q(:,ii));
    eulerot(:,ii) = wrapTo180(rotm2eul(Rot(:,:,ii),'ZYX')*180/pi); %hint for missing function: need to convert rotation matrix to euler angles (rotm2eul)
    %Note: wrapTo360 wraps 0 to 360, wrapTo180 wraps -180 to 180
end

%plot the joint positions
figure;plot(lambda,q,'LineWidth',2);  %hint: see line 28
xlabel('$\lambda$','Interpreter','latex');  %the $ and interpreter argument let you use Latex notation for labels
ylabel('$q(\lambda)$ (degrees)','Interpreter','latex');
lg = legend('1','2','3','4','5');  %use legend to label your plots
title(lg,'Joint'); %this adds a title to your legend. it can be useful for cases like this rather than writing joint 1, joint 2, etc and cluttering the legend

%plot the Euler angle representation of EE orientation
figure;plot(lambda,eulerot,'LineWidth',2);


%plot the EE position
figure;plot(lambda,Pot,'LineWidth',2);

%visual the start and end poses. Be sure to label the ***figures*** using **title**
dofbot = defineDofbotPoE();
[dofbot,~] = collisionBody(dofbot,2/100);

%%%%% Big Note: showThick expects the joint angles in radians: showThick(robotObject, jointConfiguration, collisionVisualFlag)
figure; showThick(dofbot,deg2rad(qstart),'on')
title('Start Pose');

figure; showThick(dofbot,deg2rad(qend),'on')
title('End Pose');
