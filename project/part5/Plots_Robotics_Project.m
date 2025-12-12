%% import data
test1 = readmatrix("test1_horizontal.csv", Delimiter = ",");
test2 = readmatrix("test2_vertical.csv", Delimiter = ",");
test3 = readmatrix("test3_both.csv", Delimiter = ",");
test4 = readmatrix("test4_both.csv", Delimiter = ",");

u1 = test1(:,1);
v1 = test1(:,2);
u2 = test2(:,1);
v2 = test2(:,2);
u3 = test3(:,1);
v3 = test3(:,2);
u4 = test4(:,1);
v4 = test4(:,2);

%% define constants 
%screen constants (pixels)
u0 = 320; %center of screen u
v0 = 240; %center of screen v
r0 = 50;

l = 640; %length of screen
w = 480; %width of screen 

%approximate time 
tf1 = 13.09; %(s)
tf2 = 10.20;
tf3 = 13.78; 
tf4 = 16.0;

%time vector
t1 = 0:tf1/(size(u1,1)-1):tf1;
t2 = 0:tf2/(size(u2,1)-1):tf2;
t3 = 0:tf3/(size(u3,1)-1):tf3;
t4 = 0:tf4/(size(u4,1)-1):tf4;

%intended area to keep dot in 


%% distance from center 
%1
r1_u = u1-u0;
r1_v = v1-v0;
r1 = sqrt(r1_u.^2 + r1_v.^2);

%2
r2_u = u2-u0;
r2_v = v2-v0;
r2 = sqrt(r2_u.^2 + r2_v.^2);

%3
r3_u = u3-u0;
r3_v = v3-v0;
r3 = sqrt(r3_u.^2 + r3_v.^2);

%4
r4_u = u4-u0;
r4_v = v4-v0;
r4 = sqrt(r4_u.^2 + r4_v.^2);

%% Find values over max
%1
over1 = r1>r0;
r1_over = r1(over1);
t1_over = t1(over1);
u1_over = u1(over1);
v1_over = v1(over1);

%2
over2 = r2>r0;
r2_over = r2(over2);
t2_over = t2(over2);
u2_over = u2(over2);
v2_over = v2(over2);

%3
over3 = r3>r0;
r3_over = r3(over3);
t3_over = t3(over3);
u3_over = u3(over3);
v3_over = v3(over3);

%4
over4 = r4>r0;
r4_over = r4(over4);
t4_over = t4(over4);
u4_over = u4(over4);
v4_over = v4(over4);
%% Find values within tol
%1
pass1 = r1<=r0;
r1_pass = r1(pass1);
t1_pass = t1(pass1);
u1_pass = u1(pass1);
v1_pass = v1(pass1);

%2
pass2 = r2<=r0;
r2_pass = r2(pass2);
t2_pass = t2(pass2);
u2_pass = u2(pass2);
v2_pass = v2(pass2);

%3
pass3 = r3<=r0;
r3_pass = r3(pass3);
t3_pass = t3(pass3);
u3_pass = u3(pass3);
v3_pass = v3(pass3);

%1
pass4 = r4<=r0;
r4_pass = r4(pass4);
t4_pass = t4(pass4);
u4_pass = u4(pass4);
v4_pass = v4(pass4);

%% Plots
figure(1)
plot(u1(1), v1(1), 'c*',LineWidth=4)
hold on 
scatter(u1_pass,v1_pass, 'blue')
scatter(u1_over, v1_over, 'red')
circle(u0, v0, r0);
hold off
ylim([0,w])
xlim([0,l])
grid on
title('Test 1: Centroid Path on Camera Screen')
xlabel('u (pixels)')
ylabel('v (pixels)')
legend('Starting Point', 'Path Within Tolerance', 'Path Out of Tolerance', 'Desired Location')

figure(2)
rectangle('Position', [0 0 t1(end)*1.01 r0], faceColor='g', faceAlpha=0.1, edgeColor='none')
hold on 
scatter(t1_pass, r1_pass, 'blue')
scatter(t1_over, r1_over, 'red')
hold off
xlim([0 t1(end)*1.01])
grid on 
title('Test 1: Distance from Center vs Time')
xlabel('time (s)')
ylabel('Distance (pixels)')
legend('Within Tolerance', 'Out of Tolerance')

figure(3)
plot(u2(1), v2(1), 'c*',LineWidth=4)
hold on 
scatter(u2_pass,v2_pass, 'blue')
scatter(u2_over, v2_over, 'red')
circle(u0, v0, r0);
hold off
ylim([0,w])
xlim([0,l])
grid on
title('Test 2: Path of Centroid on Camera Screen')
xlabel('u (pixels)')
ylabel('v (pixels)')
legend('Starting Point', 'Path Within Tolerance', 'Path Out of Tolerance', 'Desired Location')

figure(4)
rectangle('Position', [0 0 t2(end)*1.01 r0], faceColor='g', faceAlpha=0.1, edgeColor='none')
hold on 
scatter(t2_pass, r2_pass, 'blue')
scatter(t2_over, r2_over, 'red')
hold off
xlim([0 t2(end)*1.01])
grid on 
title('Test 2: Distance from Center vs Time')
xlabel('time (s)')
ylabel('Distance (pixels)')
legend('Within Tolerance', 'Out of Tolerance')

figure(5)
plot(u3(1), v3(1), 'c*',LineWidth=4)
hold on 
scatter(u3_pass,v3_pass, 'blue')
scatter(u3_over, v3_over, 'red')
circle(u0, v0, r0);
hold off
ylim([0,w])
xlim([0,l])
grid on
title('Test 3: Path of Centroid on Camera Screen')
xlabel('u (pixels)')
ylabel('v (pixels)')
legend('Starting Point', 'Path Within Tolerance', 'Path Out of Tolerance', 'Desired Location')

figure(6)
rectangle('Position', [0 0 t3(end)*1.01 r0], faceColor='g', faceAlpha=0.1, edgeColor='none')
hold on 
scatter(t3_pass, r3_pass, 'blue')
scatter(t3_over, r3_over, 'red')
hold off
xlim([0 t3(end)*1.01])
grid on 
title('Test 3: Distance from Center vs Time')
xlabel('time (s)')
ylabel('Distance (pixels)')
legend('Within Tolerance', 'Out of Tolerance')

figure(7)
plot(u4(1), v4(1), 'c*',LineWidth=4)
hold on 
scatter(u4_pass,v4_pass, 'blue')
scatter(u4_over, v4_over, 'red')
circle(u0, v0, r0);
hold off
ylim([0,w])
xlim([0,l])
grid on
title('Test 4: Path of Centroid on Camera Screen')
xlabel('u (pixels)')
ylabel('v (pixels)')
legend('Starting Point', 'Path Within Tolerance', 'Path Out of Tolerance', 'Desired Location')

figure(8)
rectangle('Position', [0 0 t4(end)*1.01 r0], faceColor='g', faceAlpha=0.1, edgeColor='none')
hold on 
scatter(t4_pass, r4_pass, 'blue')
scatter(t4_over, r4_over, 'red')
hold off
xlim([0 t4(end)*1.01])
grid on 
title('Test 4: Distance from Center vs Time')
xlabel('time (s)')
ylabel('Distance (pixels)')
legend('Within Tolerance', 'Out of Tolerance')







