%see effect of camera parameters
P0C = [.4;.3;4];
R0C = rot([0;0;1],45*pi/180)*rot([1;0;1],20*pi/180);
TC0 = [R0C P0C;0 0 0 1];

W = 1280; %width in pixels (1280)
H = 1024; %height in pixels (1024)
rhow = 1e-5; %width per pixel (10um)
rhoh = 1e-5; %height per pixel (10um)
f = 0.015; %focal length (0.015m)
cam = defineCam(W,H,rhow,rhoh,f);

P0 = defineTarget();

[uv,uvw,P1] = cam_image(cam,TC0,P0);

figure;plot(uv(1,:),uv(2,:),'x','LineWidth',2);
grid on; view(180,-90); axis([0 W 0 H])


function cam = defineCam(W,H,rhow,rhoh,f)
    cam.rho = [rhow;rhoh];
    cam.phih = 2*atan2(H*rhoh/2,f); %the vertical field of view angle
    cam.phiw = 2*atan2(W*rhow/2,f); %the horizontal field of view angle
    u0 = W/2;
    v0 = H/2; %the vertical optical center, assumed to be half the image plane height in pixels
    cam.uv0 = [u0;v0];
    cam.f = f;
    cam.K = [f/rhow, 0, u0;0, f/rhoh, v0;0, 0, 1]; %the intrinsic camera matrix
end

function [uv,uvw,P1] = cam_image(cam,Tco,P0)
    K=cam.K; % intrinsic camera matrix
    C=K*Tco(1:3,:); %full camera matrix

    % points in the camera frame
    P1=Tco(1:3,1:3)*P0;

    % (u',v',w') 
    uvw=C*[P0;ones(1,size(P0,2))];

    % image plane coordinate in pixels
    u=(uvw(1,:)./uvw(3,:));
    v=(uvw(2,:)./uvw(3,:));
    uv=[u;v];

    % only keep points within the image plane
    uv=uv(:,uv(1,:)<2*cam.uv0(1));
    uv=uv(:,uv(2,:)<2*cam.uv0(2));
    uv=uv(:,uv(1,:)>0);
    uv=uv(:,uv(2,:)>0);
end

function P0 = defineTarget()
    z0=0; % z-coordinate of the points (planar, so 0)
    nx1=-.1;kx=.2;nx2=.1; % arrays in x
    ny1=-.1;ky=.2;ny2=.1; % arrays in y
    px=(nx1:kx:nx2);Nx=length(px);
    py=(ny1:ky:ny2);Ny=length(py);
    Pxy=kron([zeros(1,Nx);ones(1,Nx)],py)+...
            kron(px,[ones(1,Ny);zeros(1,Ny)]);
    N = Nx*Ny;
    P0=[Pxy;z0*ones(1,Nx*Ny)]; % pack into a 2x(Nx*Ny) vector
end

function R=rot(k,theta)
    k = k / norm(k);
   %see Euler-Rodrigues lecture 4
    R = eye(3,3)+sin(theta)*hat(k)+(1-cos(theta))*hat(k)*hat(k);
end 

function khat = hat(k)
    khat=[0 -k(3) k(2); k(3) 0 -k(1); -k(2) k(1) 0];
end
