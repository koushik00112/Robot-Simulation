clc
clear
close all

%% ================= MAP SETUP =================
% Create a 50x50 grid map (0 = free, 1 = obstacle)

mapSize = 50;
map = zeros(mapSize);

% Vertical shelves (obstacles)
map(:,12)=1;
map(:,25)=1;
map(:,38)=1;

% Create aisle gaps in shelves
map(15:35,12)=0;
map(15:35,25)=0;
map(15:35,38)=0;

%% ================= ROBOT =================
% Initial robot position and orientation

robot.x = 5;
robot.y = 5;
robot.theta = 0;   % heading angle
robotRadius = 0.7;

%% ================= ITEMS =================
% Items placed ON SHELVES (fixed positions)

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

%% ================= PEOPLE =================
% Random background people

numPeople = 3;
people = rand(numPeople,2)*40+5;

% Main human (for blocking demo)
mainPerson = [30 30];
humanTriggered = false;

%% ================= VISIT ORDER =================
% Compute visiting order using nearest neighbour

order = nearestNeighbour([robot.x robot.y],items);

%% ================= LIDAR =================
% Simulated 2D LIDAR

lidarRange = 8;
angles = linspace(-pi,pi,40);

%% ================= FIGURE =================

figure
hold on
axis([1 mapSize 1 mapSize])
axis square
set(gca,'YDir','normal')

imagesc(map')
colormap(gray)

%% ================= MAIN LOOP =================

for i = 1:length(order)

    % Current target item
    itemTarget = items(order(i),:);

    % Convert shelf position → reachable free cell
    goal = getReachableGoal(map, itemTarget);

    % Plan path using A*
    path = astar(map,[robot.x robot.y],goal);

    if isempty(path)
        continue
    end

    k = 1;

    %% ===== FOLLOW PATH =====
    while k <= size(path,1)

        target = path(k,:);
        distToTarget = norm([robot.x robot.y] - target);

        % Move to next waypoint if close enough
        if distToTarget < 0.4
            k = k + 1;
            continue
        end

        %% 🔥 HUMAN INTERCEPTION LOGIC
        % Place human ON robot path once

        if ~humanTriggered && k > 5 && k+2 <= size(path,1)
            mainPerson = path(k+2,:);  % block ahead
            humanTriggered = true;
        end

        %% 🔥 HUMAN MOVEMENT (CROSSING BEHAVIOR)
        if humanTriggered

            % Human walks across aisle (right direction)
            mainPerson = mainPerson + [0.2 0];

            % Once far → remove human
            if norm(mainPerson - [robot.x robot.y]) > 6
                humanTriggered = false;
                mainPerson = [45 45]; % move away
            end
        end

        % Keep human inside bounds
        mainPerson = max(2,min(mapSize-2,mainPerson));

        %% 🚨 OBSTACLE DETECTION (HUMAN)
        if humanTriggered && norm([robot.x robot.y] - mainPerson) < 2.5

            % Clear and redraw scene
            cla
            drawScene(map, items, goal, people, mainPerson, path)

            % Display waiting message
            text(robot.x+1, robot.y+1, ...
                'WAITING FOR HUMAN TO MOVE',...
                'Color','r','FontSize',10)

            % Robot LED = RED (stopped)
            drawRobot(robot, robotRadius, 'r')

            % Show LIDAR
            drawLidar(robot, angles, lidarRange, map)

            title('🚨 HUMAN BLOCKING → ROBOT STOPPED')

            drawnow
            pause(0.1)

            continue   % HARD STOP (no movement)
        end

        %% ================= ROBOT MOTION =================
        % Smooth differential drive style motion

        dx = target(1) - robot.x;
        dy = target(2) - robot.y;
        dist = sqrt(dx^2 + dy^2);

        if dist > 0.01

            % Desired direction
            desiredTheta = atan2(dy,dx);

            % Angle error correction
            angleError = wrapToPi(desiredTheta - robot.theta);

            % Rotate smoothly
            robot.theta = robot.theta + 0.25 * angleError;

            % Move forward only when aligned
            if abs(angleError) < 0.4
                step = min(0.5, dist);
                robot.x = robot.x + step*cos(robot.theta);
                robot.y = robot.y + step*sin(robot.theta);
            end
        end

        % Snap to waypoint (prevents spinning issue)
        if dist < 0.2
            robot.x = target(1);
            robot.y = target(2);
        end

        % Keep robot inside map
        robot.x = max(1, min(mapSize, robot.x));
        robot.y = max(1, min(mapSize, robot.y));

        %% ================= RANDOM PEOPLE =================
        for p=1:numPeople
            people(p,:) = people(p,:) + randn(1,2)*0.3;
            people(p,:) = max(2,min(mapSize-2,people(p,:)));
        end

        %% ================= DRAW =================
        cla
        drawScene(map, items, goal, people, mainPerson, path)

        % Robot LED = GREEN (moving)
        drawRobot(robot, robotRadius, 'g')

        drawLidar(robot, angles, lidarRange, map)

        title('FINAL: Human Intercepts → Robot Stops → Continues')

        drawnow
        pause(0.05)

        %% ================= GOAL CHECK =================
        if norm([robot.x robot.y]-goal)<1
            simulateArm(robot)
            break
        end

    end
end

disp("All items collected!")

%% ================= DRAW FUNCTIONS =================

function drawScene(map, items, goal, people, mainPerson, path)
imagesc(map')
colormap(gray)
hold on

% Items
plot(items(:,1),items(:,2),'rx','MarkerSize',12,'LineWidth',2)

% Current goal
plot(goal(1),goal(2),'go','MarkerSize',12,'LineWidth',2)

% Random people
plot(people(:,1),people(:,2),'mo','MarkerSize',10,'LineWidth',2)

% Blocking human
plot(mainPerson(1),mainPerson(2),'ro','MarkerSize',14,'LineWidth',3)

% Path
if ~isempty(path)
    plot(path(:,1),path(:,2),'g','LineWidth',2)
end
end

function drawRobot(robot, r, ledColor)

theta = linspace(0,2*pi,40);

% Robot body
fill(robot.x + r*cos(theta),robot.y + r*sin(theta),'b')

% Heading arrow
quiver(robot.x,robot.y,cos(robot.theta),sin(robot.theta),1,'w','LineWidth',2);

% LED indicator (status)
plot(robot.x, robot.y+1.2,'o',...
    'MarkerFaceColor',ledColor,'Color',ledColor)
end

function drawLidar(robot, angles, range, map)
for a = angles
rt = robot.theta + a;

for r = 0:0.3:range
rx = robot.x + r*cos(rt);
ry = robot.y + r*sin(rt);

if rx<1 || ry<1 || rx>size(map,1) || ry>size(map,2)
break
end

if map(round(rx),round(ry))==1
break
end
end

plot([robot.x rx],[robot.y ry],'c')
end
end

%% ================= SCARA ARM =================
function simulateArm(robot)
for z=0:0.1:1
plot(robot.x,robot.y+z,'ys','MarkerSize',8,'MarkerFaceColor','y')
drawnow
pause(0.05)
end
for z=1:-0.1:0
plot(robot.x,robot.y+z,'ys','MarkerSize',8,'MarkerFaceColor','y')
drawnow
pause(0.05)
end
end

%% ================= NEAREST NEIGHBOUR =================
function order = nearestNeighbour(start,items)
remaining = items; order=[]; pos=start; indices=1:size(items,1);
while ~isempty(remaining)
dist = vecnorm(remaining-pos,2,2);
[~,idx]=min(dist);
order(end+1)=indices(idx);
pos=remaining(idx,:);
remaining(idx,:)=[];
indices(idx)=[];
end
end

%% ================= A* PATH PLANNER =================
function path = astar(map,start,goal)

start=round(start);
goal=round(goal);

moves=[1 0;-1 0;0 1;0 -1];

open=start;
cameFrom=containers.Map;

gScore=inf(size(map));
gScore(start(1),start(2))=0;

fScore=inf(size(map));
fScore(start(1),start(2))=norm(start-goal);

while ~isempty(open)

scores=arrayfun(@(i)fScore(open(i,1),open(i,2)),1:size(open,1));
[~,idx]=min(scores);

current=open(idx,:);

if isequal(current,goal)
path=reconstruct(cameFrom,current);
return
end

open(idx,:)=[];

for m=1:4

neighbor=current+moves(m,:);

if neighbor(1)<1 || neighbor(2)<1 || neighbor(1)>size(map,1) || neighbor(2)>size(map,2)
continue
end

if map(neighbor(1),neighbor(2))==1
continue
end

tent=gScore(current(1),current(2))+1;

if tent<gScore(neighbor(1),neighbor(2))

cameFrom(mat2str(neighbor))=current;
gScore(neighbor(1),neighbor(2))=tent;
fScore(neighbor(1),neighbor(2))=tent+norm(neighbor-goal);

if ~ismember(neighbor,open,'rows')
open=[open;neighbor];
end

end

end

end

path=[];
end

%% ================= PATH RECONSTRUCTION =================
function path=reconstruct(cameFrom,current)

path=current;

while true
key=mat2str(current);

if ~isKey(cameFrom,key)
break
end

current=cameFrom(key);
path=[current;path];
end

end

%% ================= GOAL FIX =================
function goal = getReachableGoal(map, item)

x=item(1);
y=item(2);

neighbors=[x+1 y; x-1 y; x y+1; x y-1];

for i=1:4
nx=neighbors(i,1);
ny=neighbors(i,2);

if nx>=1 && ny>=1 && nx<=size(map,1) && ny<=size(map,2)
if map(nx,ny)==0
goal=[nx ny];
return
end
end
end

goal=item;

end