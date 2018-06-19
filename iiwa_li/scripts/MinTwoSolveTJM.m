function    TJM=MinTwoSolveTJM()
clear
clc
file1='/home/lizq/win7share/Rx.txt';%加载txt数据
Tx=load(file1);
file2='/home/lizq/win7share/Ry.txt';
Ty=load(file2);
file3='/home/lizq/win7share/Rz.txt';
Tz=load(file3);
file4='/home/lizq/win7share/Dx.txt';
Rx=load(file4);
file5='/home/lizq/win7share/Dy.txt';
Ry=load(file5);
file6='/home/lizq/win7share/Dz.txt';
Rz=load(file6);
file7='/home/lizq/win7share/auto_calibrate_TCP.txt';
TCP=load(file7);
TON=load('/home/lizq/win7share/TON.txt');
[m,n]=size(Rx);
NDI=textread('/home/lizq/win7share/NDI.txt','',1,'delimiter',',');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Rxx=Rx(:,1);Ryy=Rx(:,2);Rzz=Rx(:,3);
for i=2:m
   for j=1:i-1
       P(i-1,j,1)=Rxx(i)-Rxx(j);
       P(i-1,j,2)=Ryy(i)-Ryy(j);
       P(i-1,j,3)=Rzz(i)-Rzz(j);
       p(i-1,j,1)=P(i-1,j,1)/(sqrt(P(i-1,j,1)^2+P(i-1,j,2)^2+P(i-1,j,3)^2));
       p(i-1,j,2)=P(i-1,j,2)/(sqrt(P(i-1,j,1)^2+P(i-1,j,2)^2+P(i-1,j,3)^2));
       p(i-1,j,3)=P(i-1,j,3)/(sqrt(P(i-1,j,1)^2+P(i-1,j,2)^2+P(i-1,j,3)^2));
   end
end
Rx1=0;Rx2=0;Rx3=0;
for i=2:m
    for j=1:i-1
        Rx1=Rx1+p(i-1,j,1);
        Rx2=Rx2+p(i-1,j,2);
        Rx3=Rx3+p(i-1,j,3);
    end
end
Rx1=Rx1/sum(1:m-1);
Rx2=Rx2/sum(1:m-1);
Rx3=Rx3/sum(1:m-1);
RX1=[Rx1;Rx2;Rx3];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Rxx=Ry(:,1);Ryy=Ry(:,2);Rzz=Ry(:,3);
for i=2:m
   for j=1:i-1
       W(i-1,j,1)=Rxx(i)-Rxx(j);
       W(i-1,j,2)=Ryy(i)-Ryy(j);
       W(i-1,j,3)=Rzz(i)-Rzz(j);
       w(i-1,j,1)=W(i-1,j,1)/(sqrt(W(i-1,j,1)^2+W(i-1,j,2)^2+W(i-1,j,3)^2));
       w(i-1,j,2)=W(i-1,j,2)/(sqrt(W(i-1,j,1)^2+W(i-1,j,2)^2+W(i-1,j,3)^2));
       w(i-1,j,3)=W(i-1,j,3)/(sqrt(W(i-1,j,1)^2+W(i-1,j,2)^2+W(i-1,j,3)^2));
   end
end
Ry1=0;Ry2=0;Ry3=0;
for i=2:m
    for j=1:i-1
        Ry1=Ry1+w(i-1,j,1);
        Ry2=Ry2+w(i-1,j,2);
        Ry3=Ry3+w(i-1,j,3);
    end
end
Ry1=Ry1/sum(1:m-1);
Ry2=Ry2/sum(1:m-1);
Ry3=Ry3/sum(1:m-1);
RY1=[Ry1;Ry2;Ry3];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Rxx=Rz(:,1);Ryy=Rz(:,2);Rzz=Rz(:,3);
for i=2:m
   for j=1:i-1
       M(i-1,j,1)=Rxx(i)-Rxx(j);
       M(i-1,j,2)=Ryy(i)-Ryy(j);
       M(i-1,j,3)=Rzz(i)-Rzz(j);
       l(i-1,j,1)=M(i-1,j,1)/(sqrt(M(i-1,j,1)^2+M(i-1,j,2)^2+M(i-1,j,3)^2));
       l(i-1,j,2)=M(i-1,j,2)/(sqrt(M(i-1,j,1)^2+M(i-1,j,2)^2+M(i-1,j,3)^2));
       l(i-1,j,3)=M(i-1,j,3)/(sqrt(M(i-1,j,1)^2+M(i-1,j,2)^2+M(i-1,j,3)^2));
   end
end
Rz1=0;Rz2=0;Rz3=0;
for i=2:m
    for j=1:i-1
        Rz1=Rz1+l(i-1,j,1);
        Rz2=Rz2+l(i-1,j,2);
        Rz3=Rz3+l(i-1,j,3);
    end
end
Rz1=Rz1/sum(1:m-1);
Rz2=Rz2/sum(1:m-1);
Rz3=Rz3/sum(1:m-1);
RZ1=[Rz1;Rz2;Rz3];
R=[RX1,RY1,RZ1];

data1=[Tx(:,1);Ty(:,1);Tz(:,1)];
data2=[Tx(:,2);Ty(:,2);Tz(:,2)];
data3=[Tx(:,3);Ty(:,3);Tz(:,3)];
data=[data1,data2,data3];%%%%%f(x,y,z)=(x-a)^2+(y-b)^2+(z-c)^2-R^2;求球心坐标和半径
f=@(p,data)(data(:,1)-p(1)).^2+(data(:,2)-p(2)).^2+(data(:,3)-p(3)).^2-p(4)^2;
p=nlinfit(data,zeros(size(data,1),1),f,[0 0 0 1]');%%%%求出球心坐标和半径
p(4,1)=1;R(4,1)=0;R(4,2)=0;R(4,3)=0;
TMO=[R,p];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%机械臂基座“J”与末端TCP“O”的齐次变换矩阵%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%读取四元数转换为旋转矩阵，m-->mm%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
x=TCP(1,4);y=TCP(1,5);z=TCP(1,6);w=TCP(1,7);
TJO(4,1)=0;TJO(4,2)=0;TJO(4,3)=0;TJO(4,4)=1;
TJO(1,:)=[2*(w*w+x*x)-1 , 2*(x*y-w*z) , 2*(x*z+w*y) , TCP(1,1)*1000];
TJO(2,:)=[2*(x*y+w*z) , 2*(w*w+y*y)-1 , 2*(y*z-w*x) , TCP(1,2)*1000];
TJO(3,:)=[2*(x*z-w*y) , 2*(y*z+w*x) , 2*(w*w+z*z)-1 , TCP(1,3)*1000];

TJM=TJO/TMO;%TJO*inv(TMO)
fid=fopen('/home/lizq/win7share/TJM.txt','wt');%改为你自己文件的位置
[m,n]=size(TJM);
for i=1:m
   for j=1:n
     if j==n
        fprintf(fid,'%f\n',double(TJM(i,j)));
     else
        fprintf(fid,'%f,',double(TJM(i,j))); 
     end
   end
end
fclose(fid);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%由NDI求TMB继而得到TOB%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%四元数转旋转矩阵,第一行为449被动刚体%%%%%%%%%%%%%%%%%%
x=NDI(1,4);y=NDI(1,5);z=NDI(1,6);w=NDI(1,7);
TMB(4,1)=0;TMB(4,2)=0;TMB(4,3)=0;TMB(4,4)=1;
TMB(1,:)=[2*(w*w+x*x)-1 , 2*(x*y-w*z) , 2*(x*z+w*y) , NDI(1,1)];
TMB(2,:)=[2*(x*y+w*z) , 2*(w*w+y*y)-1 , 2*(y*z-w*x) , NDI(1,2)];
TMB(3,:)=[2*(x*z-w*y) , 2*(y*z+w*x) , 2*(w*w+z*z)-1 , NDI(1,3)];

TOB=TMO\TMB;%inv(TMO)*TMB
fid=fopen('/home/lizq/win7share/TOB.txt','wt');%改为你自己文件的位置
[m,n]=size(TOB);
for i=1:m
   for j=1:n
     if j==n
        fprintf(fid,'%f\n',double(TOB(i,j)));
     else
        fprintf(fid,'%f,',double(TOB(i,j))); 
     end
   end
end
fclose(fid);

TBN=TOB\TON;  % TBO*TON 通过自己调整的TON获得此次标定时相对的TBN 或者通过标定的TBN计算TON，但是针太软，没法用NDI标定
fid=fopen('/home/lizq/win7share/TBN.txt','wt');%改为你自己文件的位置
[m,n]=size(TBN);
for i=1:m
   for j=1:n
     if j==n
        fprintf(fid,'%f\n',double(TBN(i,j)));
     else
        fprintf(fid,'%f,',double(TBN(i,j))); 
     end
   end
end
fclose(fid);