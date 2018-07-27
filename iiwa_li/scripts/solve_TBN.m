TON=load('/home/lizq/win7share/TON.txt');
TOB=load('/home/lizq/win7share/TOB.txt');
TCP=load('/home/lizq/win7share/auto_calibrate_TCP.txt');
NDI=textread('/home/lizq/win7share/NDI.txt','',1,'delimiter',',');

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

x=TCP(1,4);y=TCP(1,5);z=TCP(1,6);w=TCP(1,7);
TJO(4,1)=0;TJO(4,2)=0;TJO(4,3)=0;TJO(4,4)=1;
TJO(1,:)=[2*(w*w+x*x)-1 , 2*(x*y-w*z) , 2*(x*z+w*y) , TCP(1,1)*1000];
TJO(2,:)=[2*(x*y+w*z) , 2*(w*w+y*y)-1 , 2*(y*z-w*x) , TCP(1,2)*1000];
TJO(3,:)=[2*(x*z-w*y) , 2*(y*z+w*x) , 2*(w*w+z*z)-1 , TCP(1,3)*1000];

x=NDI(1,4);y=NDI(1,5);z=NDI(1,6);w=NDI(1,7);
TMB(4,1)=0;TMB(4,2)=0;TMB(4,3)=0;TMB(4,4)=1;
TMB(1,:)=[2*(w*w+x*x)-1 , 2*(x*y-w*z) , 2*(x*z+w*y) , NDI(1,1)];
TMB(2,:)=[2*(x*y+w*z) , 2*(w*w+y*y)-1 , 2*(y*z-w*x) , NDI(1,2)];
TMB(3,:)=[2*(x*z-w*y) , 2*(y*z+w*x) , 2*(w*w+z*z)-1 , NDI(1,3)];

TJM=TJO * TOB / TMB;
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