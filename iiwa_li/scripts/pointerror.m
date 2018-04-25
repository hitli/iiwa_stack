clc
clear
a1=[-52.2300,64.4000,-1181.0500,-0.4117,-0.1769,-0.8114,0.3749];
a2=[-52.3600, 64.4300, -1181.1899, -0.4115, -0.1773, -0.8114, 0.3749];
d1=a1(1:3);
d2=a2(1:3);
r1(1)=a1(7);
r1(2:4)=a1(4:6);
r2(1)=a2(7);
r2(2:4)=a2(4:6);
disp('位置偏差');
disp(norm(d1-d2));
q=quatdivide(r2,r1);%第一位为实部
q
disp('角度偏差');
tht=acos(q(1))*2*180/pi;
disp(tht);