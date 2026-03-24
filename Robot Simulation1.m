clc
clear
close all

%% ================= MAP =================
mapSize = 50;
map = zeros(mapSize);

for y = 10:10:40
    map(y,5:45) = 1;
end

for y = 10:10:40
    map(y,20:25) = 0;
end

%% ================= ROBOT =================
robot.x = 5;
robot.y = 5;
robot.theta = 0;
robotRadius = 0.7;

%% ================= METRICS =================
totalDistance = 0;
stopCount = 0;
startTime = tic;
lastReplanTime = 0;

%% ================= TRAJECTORY =================
trajectory = [];

%% ================= ITEMS =================
items = [
    12 12
    25 12
    38 12
    12 25
    25 25
    38 25
    12 38
    25 38
    38 38
];

collected = false(size(items,1),1);

%% ================= CHECKOUT =================
checkout = [45 5];

%% ================= HUMANS =================
numPeople = 5;
people = rand(numPeople,2)*30 + 10;
vel = randn(numPeople,2)*0.05;

%% ================= ORDER =================
order = nearestNeighbour([robot.x robot.y],items);

%% ================= LIDAR =================
angles = linspace(-2*pi/3, 2*pi/3, 60);
lidarRange = 10;

%% ================= FIGURE =================
figure
axis([1 mapSize 1 mapSize])
axis square
set(gca,'YDir','normal')
set(gcf,'Color','k')

%% ================= SHOPPING LOOP =================
for i = 1:length(order)

    idx = order(i);
    if collected(idx), continue, end

    goal = getReachableGoal(map, items(idx,:));
    path = astar(map,[robot.x robot.y],goal);
    if isempty(path), continue, end

    k = 1;

    while k <= size(path,1)

        %% ===== HUMAN MOTION =====
        for p = 1:numPeople
            vel(p,:) = vel(p,:) + 0.02*randn(1,2);

            speed = norm(vel(p,:));
            maxSpeed = 0.15;

            if speed > maxSpeed
                vel(p,:) = vel(p,:) / speed * maxSpeed;
            end

            people(p,:) = people(p,:) + vel(p,:);

            for d = 1:2
                if people(p,d) < 2 || people(p,d) > mapSize-2
                    vel(p,d) = -vel(p,d);
                end
            end

            people(p,:) = max(2,min(mapSize-2,people(p,:)));
        end

        %% ===== RANDOM BLOCKING =====
        if rand < 0.03 && k < size(path,1)
            people(1,:) = path(min(k+3,size(path,1)),:);
        end

        target = path(k,:);

        if norm([robot.x robot.y] - target) < 0.4
            k = k + 1;
            continue
        end

        %% ===== DETECTION =====
        isBlocked = lidarDetectHumans(robot,people,angles,lidarRange);

        %% ===== REPLAN =====
        if isBlocked && toc(startTime) - lastReplanTime > 1

            stopCount = stopCount + 1;
            lastReplanTime = toc(startTime);

            tempMap = map;

            for h = 1:size(people,1)
                hx = round(people(h,1));
                hy = round(people(h,2));

                for dx=-1:1
                    for dy=-1:1
                        nx = hx + dx;
                        ny = hy + dy;

                        if nx>=1 && ny>=1 && nx<=mapSize && ny<=mapSize
                            tempMap(nx,ny) = 1;
                        end
                    end
                end
            end

            newPath = astar(tempMap,[robot.x robot.y],goal);

            if ~isempty(newPath)
                path = newPath;
                k = min(3,size(path,1));
            else
                continue
            end
        end

        %% ===== SAFE MOTION =====
        dx = target(1) - robot.x;
        dy = target(2) - robot.y;
        dist = sqrt(dx^2 + dy^2);

        if dist > 0.01
            desiredTheta = atan2(dy,dx);
            angleError = wrapToPi(desiredTheta - robot.theta);

            robot.theta = robot.theta + 0.15 * angleError;

            if abs(angleError) < 0.6
                step = min(0.5, dist);

                nextX = robot.x + step*cos(robot.theta);
                nextY = robot.y + step*sin(robot.theta);

                ix = round(nextX);
                iy = round(nextY);

                if ix>=1 && iy>=1 && ix<=mapSize && iy<=mapSize
                    if map(ix,iy) == 0
                        robot.x = nextX;
                        robot.y = nextY;
                        totalDistance = totalDistance + step;
                    else
                        path = astar(map,[robot.x robot.y],goal);
                        k = 1;
                        continue
                    end
                end
            end
        end

        trajectory = [trajectory; robot.x robot.y];

        %% ===== DRAW =====
        cla
        drawScene(map,items,goal,people,path,trajectory,collected,checkout)

        text(2,48,"Shopping... Items Left: " + sum(~collected),'Color','y')

        drawRobot(robot,robotRadius)
        drawLidar(robot,angles,lidarRange,map)

        title('🛒 Shopping Mode')

        drawnow
        pause(0.05)

        %% ===== PICK ITEM =====
        if norm([robot.x robot.y]-goal)<1
            collected(idx) = true;
            break
        end
    end
end

%% ================= GO TO CHECKOUT =================
disp("Going to checkout...")

goal = checkout;
path = astar(map,[robot.x robot.y],goal);

if ~isempty(path)

    k = 1;

    while k <= size(path,1)

        %% ===== HUMAN MOTION =====
        for p = 1:numPeople
            vel(p,:) = vel(p,:) + 0.02*randn(1,2);

            speed = norm(vel(p,:));
            maxSpeed = 0.15;

            if speed > maxSpeed
                vel(p,:) = vel(p,:) / speed * maxSpeed;
            end

            people(p,:) = people(p,:) + vel(p,:);

            for d = 1:2
                if people(p,d) < 2 || people(p,d) > mapSize-2
                    vel(p,d) = -vel(p,d);
                end
            end

            people(p,:) = max(2,min(mapSize-2,people(p,:)));
        end

        target = path(k,:);

        if norm([robot.x robot.y] - target) < 0.4
            k = k + 1;
            continue
        end

        %% ===== DETECTION =====
        isBlocked = lidarDetectHumans(robot,people,angles,lidarRange);

        %% ===== REPLAN =====
        if isBlocked && toc(startTime) - lastReplanTime > 1

            stopCount = stopCount + 1;
            lastReplanTime = toc(startTime);

            tempMap = map;

            for h = 1:size(people,1)
                hx = round(people(h,1));
                hy = round(people(h,2));

                for dx=-1:1
                    for dy=-1:1
                        nx = hx + dx;
                        ny = hy + dy;

                        if nx>=1 && ny>=1 && nx<=mapSize && ny<=mapSize
                            tempMap(nx,ny) = 1;
                        end
                    end
                end
            end

            newPath = astar(tempMap,[robot.x robot.y],goal);

            if ~isempty(newPath)
                path = newPath;
                k = min(3,size(path,1));
            else
                continue
            end
        end

        %% ===== SAFE MOTION =====
        dx = target(1) - robot.x;
        dy = target(2) - robot.y;
        dist = sqrt(dx^2 + dy^2);

        if dist > 0.01
            desiredTheta = atan2(dy,dx);
            angleError = wrapToPi(desiredTheta - robot.theta);

            robot.theta = robot.theta + 0.15 * angleError;

            if abs(angleError) < 0.6
                step = min(0.5, dist);

                nextX = robot.x + step*cos(robot.theta);
                nextY = robot.y + step*sin(robot.theta);

                ix = round(nextX);
                iy = round(nextY);

                if ix>=1 && iy>=1 && ix<=mapSize && iy<=mapSize
                    if map(ix,iy) == 0
                        robot.x = nextX;
                        robot.y = nextY;
                        totalDistance = totalDistance + step;
                    else
                        path = astar(map,[robot.x robot.y],goal);
                        k = 1;
                        continue
                    end
                end
            end
        end

        trajectory = [trajectory; robot.x robot.y];

        %% ===== DRAW =====
        cla
        drawScene(map,items,goal,people,path,trajectory,collected,checkout)

        text(2,48,"Going to Checkout...",'Color','y')

        drawRobot(robot,robotRadius)
        drawLidar(robot,angles,lidarRange,map)

        title('🧾 Checkout Mode')

        drawnow
        pause(0.05)

        if norm([robot.x robot.y]-goal) < 1
            disp("Reached Checkout ✅")
            break
        end
    end
end

%% ================= FUNCTIONS =================
function isBlocked = lidarDetectHumans(robot, people, angles, range)
isBlocked = false;
for p = 1:size(people,1)
    if norm([robot.x robot.y] - people(p,:)) < 2
        isBlocked = true;
        return
    end
end
end

function drawScene(map, items, goal, people, path, trajectory, collected, checkout)
imagesc(map'); colormap(gray); hold on

rectangle('Position',[checkout(1)-3 checkout(2)-2 6 4],'FaceColor',[1 0.3 0.3])
text(checkout(1)-2,checkout(2),'CHECKOUT','Color','w')

for i=1:size(items,1)
    if ~collected(i)
        rectangle('Position',[items(i,1)-0.5 items(i,2)-0.5 1 1],...
            'FaceColor',[0.9 0.7 0.2],'EdgeColor','none')
    end
end

plot(people(:,1),people(:,2),'ro','MarkerFaceColor','r','MarkerSize',8)

if ~isempty(path), plot(path(:,1),path(:,2),'g','LineWidth',2), end
if ~isempty(trajectory), plot(trajectory(:,1),trajectory(:,2),'c','LineWidth',2), end
end

function drawRobot(robot, r)
theta = linspace(0,2*pi,40);
fill(robot.x + r*cos(theta),robot.y + r*sin(theta),[0 0.8 0])
quiver(robot.x,robot.y,cos(robot.theta),sin(robot.theta),1,'Color','w','LineWidth',2)
end

function drawLidar(robot, angles, range, map)
for a = angles
rt = robot.theta + a;
for r = 0:0.3:range
rx = robot.x + r*cos(rt);
ry = robot.y + r*sin(rt);
if rx<1 || ry<1 || rx>size(map,1) || ry>size(map,2), break, end
if map(round(rx),round(ry))==1, break, end
end
plot([robot.x rx],[robot.y ry],'c')
end
end

function order = nearestNeighbour(start,items)
remaining = items; order=[]; pos=start; idx=1:size(items,1);
while ~isempty(remaining)
dist = vecnorm(remaining-pos,2,2);
[~,i]=min(dist);
order(end+1)=idx(i);
pos=remaining(i,:);
remaining(i,:)=[];
idx(i)=[];
end
end

function path = astar(map,start,goal)
start=round(start); goal=round(goal);
moves=[1 0;-1 0;0 1;0 -1];

open=start;
cameFrom=containers.Map;

g=inf(size(map)); g(start(1),start(2))=0;
f=inf(size(map)); f(start(1),start(2))=norm(start-goal);

while ~isempty(open)
[~,i]=min(arrayfun(@(k)f(open(k,1),open(k,2)),1:size(open,1)));
cur=open(i,:);
if isequal(cur,goal), path=reconstruct(cameFrom,cur); return, end
open(i,:)=[];

for m=1:4
nb=cur+moves(m,:);
if any(nb<1) || nb(1)>size(map,1) || nb(2)>size(map,2), continue, end
if map(nb(1),nb(2))==1, continue, end

tent=g(cur(1),cur(2))+1;
if tent<g(nb(1),nb(2))
cameFrom(mat2str(nb))=cur;
g(nb(1),nb(2))=tent;
f(nb(1),nb(2))=tent+norm(nb-goal);
if ~ismember(nb,open,'rows'), open=[open;nb]; end
end
end
end
path=[];
end

function path=reconstruct(cameFrom,current)
path=current;
while true
k=mat2str(current);
if ~isKey(cameFrom,k), break, end
current=cameFrom(k);
path=[current;path];
end
end

function goal = getReachableGoal(map,item)
x=item(1); y=item(2);
nbr=[x+1 y;x-1 y;x y+1;x y-1];
for i=1:4
nx=nbr(i,1); ny=nbr(i,2);
if nx>=1 && ny>=1 && nx<=size(map,1) && ny<=size(map,2)
if map(nx,ny)==0, goal=[nx ny]; return, end
end
end
goal=item;
end
