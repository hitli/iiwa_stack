
function hand_eye_calibration
robot_data = load('/home/lizq/win7share/robot_data.txt');
robot_data(:,1:3) = robot_data(:,1:3)*1000;%单位换算为mm
ndi_data = load('/home/lizq/win7share/ndi_data.txt');
count = 0;

for i = 1:size(robot_data,1)%获得所有旋转矩阵
    jo = quat2dcm(quatnormalize(circshift(robot_data(i,4:7),[0,1])));  % circshift(robot_data(i,4:7),[0,1])变换四元数位置，把实部放在第一位
    bm = inv(quat2dcm(quatnormalize(circshift(ndi_data(i,4:7),[0,1]))));  % tjo*tob*tbm=tjm=tjo1*tob*tbm1 所以要将tmb求逆
    tjo(:,:,i) = [jo,robot_data(i,1:3)';0 0 0 1];
    tbm(:,:,i) = [bm,ndi_data(i,1:3)';0 0 0 1];
end
for i = 1:size(robot_data,1)  %tjo1逆*tjo*tob=tob*tbm1*tbm逆
    for j = 1:size(robot_data,1) %即AX=XB
    count = count + 1;
    B(:,:,count) = tbm(:,:,i)/tbm(:,:,j);%B=tbm1*tbm逆??
    A(:,:,count) = tjo(:,:,i)\tjo(:,:,j);%A=tjo1逆*tjo
    end
end
M = zeros(3,3);
C = [];
d = [];
ind = [];
valid_count = 0;
for i = 1:size(A,3)
    alpha(:,:,i) = calculate_log(A(1:3,1:3,i));
    beta(:,:,i) = calculate_log(B(1:3,1:3,i));
    alpha_vet(:,:,i) = extract_vect(alpha(:,:,i));
    beta_vet(:,:,i) = extract_vect(beta(:,:,i));
    if (abs(norm(alpha_vet(:,:,i))-norm(beta_vet(:,:,i)))<0.0001)  % 0.0001 is a scalar to control the quality of the two vector.
    t_ = beta(:,:,i)*alpha(:,:,i)';
    if (~isnan(t_(1)))
        ind = [ind,i];
        M = M + beta_vet(:,:,i)*alpha_vet(:,:,i)';
        valid_count = valid_count+1;
    end
    end
end
disp(['total count:=',num2str(count)]);
disp(['valid count:=', num2str(valid_count)]);
R_est = (M'*M)^(-1/2)*M'
A = A(:,:,ind);
B = B(:,:,ind);
for i =1:size(A,3)
    C = [C; eye(3)-A(1:3,1:3,i)];
    d = [d;A(1:3,4,i)-R_est*B(1:3,4,i)];
end
b_est = (C'*C)^(-1)*C'*d
%norm(b_est)
TOB = [R_est,b_est;0 0 0 1]

%fid=fopen('/home/lizq/win7share/TOB.txt','wt');%改为你自己文件的位置
% [m,n]=size(TOB);
% for i=1:m
%    for j=1:n
%      if j==n
%         fprintf(fid,'%f\n',double(TOB(i,j)));
%      else
%         fprintf(fid,'%f,',double(TOB(i,j))); 
%      end
%    end
% end
% fclose(fid);

tjm = zeros(4,4);
for i = 1:size(robot_data,1)
    tjo(:,:,i) * TOB *tbm(:,:,i)
    tjm =tjm + tjo(:,:,i) * TOB *tbm(:,:,i);
end
TJM = tjm/size(robot_data,1)

%fid=fopen('/home/lizq/win7share/TJM.txt','wt');%改为你自己文件的位置
% [m,n]=size(TJM);
% for i=1:m
%    for j=1:n
%      if j==n
%         fprintf(fid,'%f\n',double(TJM(i,j)));
%      else
%         fprintf(fid,'%f,',double(TJM(i,j))); 
%      end
%    end
% end
% fclose(fid);

% TON=load('/home/lizq/win7share/TON.txt');
% TBN=TOB\TON;  % TBO*TON 通过自己调整的TON获得此次标定时相对的TBN 或者通过标定的TBN计算TON，但是针太软，没法用NDI标定
% fid=fopen('/home/lizq/win7share/TBN.txt','wt');%改为你自己文件的位置
% [m,n]=size(TBN);
% for i=1:m
%    for j=1:n
%      if j==n
%         fprintf(fid,'%f\n',double(TBN(i,j)));
%      else
%         fprintf(fid,'%f,',double(TBN(i,j))); 
%      end
%    end
% end
% fclose(fid);

end

function vect = extract_vect(mat)
vect = [mat(3,2),mat(1,3),mat(2,1)]';
end

function alpha = calculate_log(R)
trace_ = R(1,1)+R(2,2)+R(3,3);
psi = acos((trace_-1)/2);
alpha = psi*(R-R')/(2*sin(psi));
end
