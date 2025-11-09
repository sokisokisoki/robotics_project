qstart_1 = [90; 90; 90; 90; 90];
qend_1 = [30; 45; 60; 15; 90];

qstart_2 = [30; 45; 60; 15; 90];
qend_2 = [30; 160; 20; 0; 0];

qstart_3 = [30; 160; 20; 0; 0];
qend_3 = [0; 130; 10; 0; 0];

qstart_4 = [0; 130; 10; 0; 0];
qend_4 = [30; 45; 60; 15; 90];

% same as qstart_1 and qend_1, but with higher N
qstart_star = [90; 90; 90; 90; 90];
qend_star = [30; 45; 60; 15; 90];

qstart = [90; 90; 90; 90; 90]; %initial joint configuration in degrees
qend = [30; 45; 60; 15; 90]; %final joint configuration in degrees

N = 10; %the number of sample points along the path
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

fprintf('q = [\n');
for i = 1:size(q,2)
    fprintf('  [');
    fprintf('%.4f, ', q(1:end-1,i));
    fprintf('%.4f],\n', q(end,i));
end
fprintf(']\n');