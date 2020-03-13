    clear
clc

time=6;
f=1000;
time_run=time*f;
x=zeros(2,time*f);

% Lib
j=1;
for i=0:1:f-1
    x(1,j)=-0.1*sin(((pi/6)*i)/f);
    x(2,j)=-0.1*cos(((pi/6)*i)/(f));
    j=j+1;
end

for i=f-1:-1:0
    x(1,j)=-0.1*sin(((pi/6)*i)/f);
    x(2,j)=-0.1*cos(((pi/6)*i)/(f));
    j=j+1;
end

for i=0:1:f-1
    x(1,j)=-0.1*sin(((-pi/6)*i)/f);
    x(2,j)=-0.1*cos(((-pi/6)*i)/(f));
    j=j+1;
end

for i=f-1:-1:0
    x(1,j)=-0.1*sin(((-pi/6)*i)/f);
    x(2,j)=-0.1*cos(((-pi/6)*i)/(f));
    j=j+1;
end
% 
% figure
% plot(x(1,:),x(2,:));

% Connection

% a*sin(theta)-0.06=0;
%a*cos(theta)-0.06=-0.1;
%a=0.0721;
%theta=-391*pi/1250;

%j=1;
for i=0:1:f-1
    x(1,j)=-0.0721*sin((-((pi/10)*i)/f)-391*pi/1250)-0.06;
    x(2,j)=-0.0721*cos((-((pi/10)*i)/f)-391*pi/1250)-0.06;
    j=j+1;
end

% figure
% plot(x(1,1:1000),x(2,1:1000));


% Nose

%a*sin(theta)=0.0094;
%a*cos(theta)-0.0795=-0.0795;
%a=-0.0094;
%theta=-90;

%j=1;
for i=0:1:f-1
    x(1,j)=-0.0094*sin((-((pi)*i)/f)-pi/2);
    x(2,j)=-0.0094*cos((-((pi)*i)/f)-pi/2)-0.0795;
    j=j+1;
end

% figure
% plot(x(1,1:1000),x(2,1:1000));

% Connection

%a*sin(theta)+0.06=-0.0094;
%a*cos(theta)-0.0795=-0.0795;
%a=-0.0694;
%theta=90;

%j=1;
for i=0:1:f-1
    x(1,j)=-0.0694*sin((((pi/6)*i)/f)+pi/2)+0.06;
    x(2,j)=-0.0694*cos((((pi/6)*i)/f)+pi/2)-0.0795;
    j=j+1;
end

% figure
% plot(x(1,1:1000),x(2,1:1000));

% Right eye

%a*sin(theta)+0.0094=-0.0001;
%a*cos(theta)-0.0448=-0.0448;
%a=-0.0095;
%theta=90;

%j=1;
for i=0:1:f-1
    x(1,j)=-0.0095*sin((((2*pi)*i)/f)+pi/2)+0.0094;
    x(2,j)=-0.0095*cos((((2*pi)*i)/f)+pi/2)-0.0448;
    j=j+1;
end

% Left eye

%a*sin(theta)-0.0094=-0.0001;
%a*cos(theta)-0.0448=-0.0448;
%a=0.0093;
%theta=90;

%j=1;
for i=0:1:f-1
    x(1,j)=0.0093*sin((((2*pi)*i)/f)+pi/2)-0.0094;
    x(2,j)=0.0093*cos((((2*pi)*i)/f)+pi/2)-0.0449;
    j=j+1;
end

% Connection

%a*sin(theta)+0.09=-0.0001;
%a*cos(theta)-0.0447=-0.0448;
%a=-0.09001;
%theta=90;

%j=1;
for i=0:1:f-1
    x(1,j)=-0.0901*sin((((pi/5)*i)/f)+pi/2)+0.09;
    x(2,j)=-0.0901*cos((((pi/5)*i)/f)+pi/2)-0.0448;
    j=j+1;
end

% % Face
% 2.8
%a*sin(theta)+0=0.0509;
%a*cos(theta)-0.04=0.0364;
% a=0.0918;
% theta=0.5876;
% 
% %j=1;
% for i=0:1:f-1
%     x(1,j)=a*sin((((2*pi)*i)/f)+theta);
%     x(2,j)=a*cos((((2*pi)*i)/f)+theta)-0.04;
%     j=j+1;
% end

% 
%a*sin(theta)+0=0.0449;
%a*cos(theta)-0.08=0.0332;
% a=0.0832;
% theta=0.3218;
% 
% %j=1;
% for i=0:1:f-1
%     x(1,j)=a*sin((((2*pi)*i)/f)+theta);
%     x(2,j)=a*cos((((2*pi)*i)/f)+theta)-0.06;
%     j=j+1;
% end

a=0.0701;
theta=0.2465;
a=0.0702;
theta=0.246;

%j=1;
for i=0:1:f-1
    x(1,j)=a*sin((((2*pi)*i)/f)+theta);
    x(2,j)=a*cos((((2*pi)*i)/f)+theta)-0.06;
    j=j+1;
end


figure
plot(x(1,:),x(2,:))
