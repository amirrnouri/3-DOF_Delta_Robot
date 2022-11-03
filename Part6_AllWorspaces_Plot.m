clc
close all
load('top_nonoconst_shell_ywork.mat')
load('top_nonoconst_shell_zwork.mat')
load('top_nonoconst_shell_xwork.mat')

xtop=1000*transpose(top_nonoconst_shell_xwork);
ytop=1000*transpose(top_nonoconst_shell_ywork);
ztop=1000*transpose(top_nonoconst_shell_zwork);

xg = linspace(min(xtop), max(xtop), 500); % coordinates of grid
yg = linspace(min(ytop), max(ytop), 500); % coordinates of grid
[Xg, Yg] = meshgrid(xg, yg); % X and Y grids
Zg = griddata(xtop, ytop, ztop, Xg, Yg);
subplot(3,1,1)
 mesh(Xg/1000, Yg/1000, Zg/1000);

load('body_nonoconst_shell_xwork.mat')
load('body_nonoconst_shell_zwork.mat')
load('body_nonoconst_shell_ywork.mat')
     x=1000*transpose(body_nonoconst_shell_xwork);
     y=1000*transpose(body_nonoconst_shell_ywork);
     z=1000*transpose(body_nonoconst_shell_zwork);

for i=1:size(x)
    xinitail=x(i);
    i;
    for j=i+1:size(x)
    xsec=x(j);
    if xsec==xinitail
        if x(j)>0
        x(j)=x(j)-1;
        else
        x(j)=x(j)+1;
        end
    end
    end
end


for i=1:size(y)
    ;
    yinitail=y(i);
    for j=i+1:size(y)
        
    ysec=y(j);
    if ysec==yinitail
        if y(j)>0
        y(j)=y(j)-1;
        else
        y(j)=y(j)+1;
        end

    end
    end
end

xyz=[x y z];

xg = linspace(min(x), max(x), 1000); % coordinates of grid
yg = linspace(min(x), max(x),1000); % coordinates of grid


[Xg, Yg] = meshgrid(xg, yg); % X and Y grids

Zg = griddata(x, y, z, Xg, Yg);
hold on
mesh(Xg/1000, Yg/1000, Zg/1000);
% 

grid on
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')

fig=gcf;
fig.Position(3:4)=[550,400];
view([50,50,50])
xlim([-0.5 0.5])
ylim([-0.5 0.5])
zlim([-0.7 -0.1])
title('Workspace of manipulator with \theta_1 limitation (a)')

%% 2nd

load('top_noconst_shell_ywork.mat')
load('top_noconst_shell_zwork.mat')
load('top_noconst_shell_xwork.mat')

xtop=1000*transpose(top_noconst_shell_xwork);
ytop=1000*transpose(top_noconst_shell_ywork);
ztop=1000*transpose(top_noconst_shell_zwork);

xg = linspace(min(xtop), max(xtop), 500); % coordinates of grid
yg = linspace(min(ytop), max(ytop), 500); % coordinates of grid
[Xg, Yg] = meshgrid(xg, yg); % X and Y grids
Zg = griddata(xtop, ytop, ztop, Xg, Yg);
subplot(3,1,2)
 mesh(Xg/1000, Yg/1000, Zg/1000);



load('body_noconst_shell_xwork.mat')
load('body_noconst_shell_zwork.mat')
load('body_noconst_shell_ywork.mat')
     x=1000*transpose(body_noconst_shell_xwork);
     y=1000*transpose(body_noconst_shell_ywork);
     z=1000*transpose(body_noconst_shell_zwork);

for i=1:size(x)
    xinitail=x(i);
    i;
    for j=i+1:size(x)
    xsec=x(j);
    if xsec==xinitail
        if x(j)>0
        x(j)=x(j)-1;
        else
        x(j)=x(j)+1;
        end
    end
    end
end


for i=1:size(y)
    ;
    yinitail=y(i);
    for j=i+1:size(y)
        
    ysec=y(j);
    if ysec==yinitail
        if y(j)>0
        y(j)=y(j)-1;
        else
        y(j)=y(j)+1;
        end

    end
    end
end


xyz=[x y z];

xg = linspace(min(x), max(x), 1000); % coordinates of grid
yg = linspace(min(x), max(x),1000); % coordinates of grid


[Xg, Yg] = meshgrid(xg, yg); % X and Y grids

Zg = griddata(x, y, z, Xg, Yg);
hold on
mesh(Xg/1000, Yg/1000, Zg/1000);
% 

grid on
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')

fig=gcf;
fig.Position(3:4)=[550,400];
view([50,50,50])
xlim([-0.5 0.5])
ylim([-0.5 0.5])
zlim([-0.7 -0.1])
title('Workspace of manipulator with \theta_1 and \phi limitation (b)')

%% 3rd PLot

load('top_allconst_shell_ywork.mat')
load('top_allconst_shell_zwork.mat')
load('top_allconst_shell_xwork.mat')

xtop=1000*transpose(top_allconst_shell_xwork);
ytop=1000*transpose(top_allconst_shell_ywork);
ztop=1000*transpose(top_allconst_shell_zwork);

xg = linspace(min(xtop), max(xtop), 500); % coordinates of grid
yg = linspace(min(ytop), max(ytop), 500); % coordinates of grid
[Xg, Yg] = meshgrid(xg, yg); % X and Y grids
Zg = griddata(xtop, ytop, ztop, Xg, Yg);
subplot(3,1,3)
 mesh(Xg/1000, Yg/1000, Zg/1000);

load('body_allconst_shell_xwork.mat')
load('body_allconst_shell_zwork.mat')
load('body_allconst_shell_ywork.mat')
x=1000*transpose(body_allconst_shell_xwork);
y=1000*transpose(body_allconst_shell_ywork);
z=1000*transpose(body_allconst_shell_zwork);


for i=1:size(x)
    xinitail=x(i);
    i;
    for j=i+1:size(x)
    xsec=x(j);
    if xsec==xinitail
        if x(j)>0
        x(j)=x(j)-0.2;
        else
        x(j)=x(j)+0.2;
        end
    end
    end
end


for i=1:size(y)
    ;
    yinitail=y(i);
    for j=i+1:size(y)
        
    ysec=y(j);
    if ysec==yinitail
        if y(j)>0
        y(j)=y(j)-0.2;
        else
        y(j)=y(j)+0.2;
        end

    end
    end
end

xyz=[x y z];

xg = linspace(min(x), max(x), 1000); % coordinates of grid
yg = linspace(min(y), max(y),1000); % coordinates of grid




[Xg, Yg] = meshgrid(xg, yg); % X and Y grids

Zg = griddata(x, y, z, Xg, Yg);
hold on
mesh(Xg/1000, Yg/1000, Zg/1000);


grid on
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')
fig=gcf;
fig.Position(3:4)=[550,450];
view([50,50,50])
xlim([-0.5 0.5])
ylim([-0.5 0.5])
zlim([-0.7 -0.1])

title('Workspace of manipulator with all limitations (c)')








