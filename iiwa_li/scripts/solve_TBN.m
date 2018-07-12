TON=load('/home/lizq/win7share/TON.txt');
TOB=load('/home/lizq/win7share/TOB.txt');

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