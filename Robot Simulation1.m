clc
clear
close all

%% ================= MAP =================
% Create grid map (0 = free space, 1 = obstacle/shelf)

mapSize = 50;
map = zeros(mapSize);

% Vertical shelves
map(:,12)=1;
map(:,25)=1;
map(:,38)=1;

% Open aisle gaps in shelves
map(15:35,12)=0;
map(15:35,25)=0;
map(15:35,38)=0;

%% ================= ROBOT =================
% Robot initial state

robot.x = 5;          % X position
robot.y = 5;          % Y position
robot.theta = 0;      % Orientation (heading angle)
robotRadius = 0.7;    % Size of robot

%% ================= METRICS =================
% Performance tracking

totalDistance = 0;    % Distance traveled
stopCount = 0;        % Number of stops due to humans
startTime = tic;      % Start timer

%% ================= TRAJECTORY =================
% Store actual path followed by robot

trajectory = [];

%% ================= ITEMS =================
% Fixed item locations on shelves

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

% Track which items are collected (avoid index errors)
collected = false(size(items,1),1);

%% ================= PEOPLE =================
% Random moving humans (purple)

numPeople = 3;
people = rand(numPeople,2)*40+5;

% Main human (red) used for interception demo
mainPerson = [30 30];

interceptCount = 0;
maxIntercepts = 4;   % limit number of forced interceptions

%% ================= ORDER =================
% Compute visiting order using Nearest Neighbour

order = nearestNeighbour([robot.x robot.y],items);

%% ================= LIDAR =================
% Simulated LIDAR parameters

lidarRange = 8;
angles = linspace(-pi,pi,40);

%% ================= FIGURE =================
% Initialize visualization

figure
hold on
axis([1 mapSize 1 mapSize])
axis square
set(gca,'YDir','normal')

imagesc(map')
colormap(gray)

%% ================= MAIN LOOP =================
% Loop through all items in computed order

for i = 1:length(order)

    idx = order(i);

    % Skip already collected items
    if collected(idx)
        continue
    end

    % Target item
    itemTarget = items(idx,:);

    % Convert shelf location → reachable free cell
    goal = getReachableGoal(map, itemTarget);

    % Plan path using A*
    path = astar(map,[robot.x robot.y],goal);

    if isempty(path), continue, end

    k = 1;

    %% ===== PATH FOLLOWING LOOP =====
    while k <= size(path,1)

        target = path(k,:);

        % Distance to current waypoint
        distToTarget = norm([robot.x robot.y] - target);

        % Move to next waypoint if close
        if distToTarget < 0.4
            k = k + 1;
            continue
        end

        %% ===== HUMAN INTERCEPTION =====
        % Place blocking human ahead on path

        if interceptCount < maxIntercepts && k > 5 && k+3 <= size(path,1)
            mainPerson = path(k+3,:);
            interceptCount = interceptCount + 1;
        end

        %% ===== HUMAN MOVEMENT =====
        % Main human moves across aisle

        mainPerson = mainPerson + [0.15 0];
        mainPerson = max(2,min(mapSize-2,mainPerson));

        % Random movement of other humans
        for p=1:numPeople
            people(p,:) = people(p,:) + randn(1,2)*0.3;
            people(p,:) = max(2,min(mapSize-2,people(p,:)));
        end

        %% ===== HUMAN DETECTION =====
        isBlocked = false;

        % Check main human
        if norm([robot.x robot.y] - mainPerson) < 2.5
            isBlocked = true;
        end

        % Check all other humans
        for p=1:numPeople
            if norm([robot.x robot.y] - people(p,:)) < 2.5
                isBlocked = true;
                break
            end
        end

        %% ===== REPLANNING =====
        if isBlocked

            stopCount = stopCount + 1;

            % Create temporary map including humans as obstacles
            tempMap = map;

            humans = [mainPerson; people];

            % Add safety radius around humans
            for h = 1:size(humans,1)

                hx = round(humans(h,1));
                hy = round(humans(h,2));

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

            % Recompute path
            newPath = astar(tempMap,[robot.x robot.y],goal);

            if ~isempty(newPath)
                path = newPath;
                k = 1;   % restart path tracking
            else
                % No path → wait
                cla
                drawScene(map,items,goal,people,mainPerson,path,trajectory)

                text(robot.x+1,robot.y+1,'WAITING FOR HUMAN','Color','r')

                drawRobot(robot,robotRadius,'r')
                drawLidar(robot,angles,lidarRange,map)

                title('🚨 BLOCKED')

                drawnow
                pause(0.1)
                continue
            end
        end

        %% ===== ROBOT MOTION =====
        % Compute direction to target

        dx = target(1) - robot.x;
        dy = target(2) - robot.y;

        dist = sqrt(dx^2 + dy^2);

        if dist > 0.01

            % Desired heading
            desiredTheta = atan2(dy,dx);

            % Angular error
            angleError = wrapToPi(desiredTheta - robot.theta);

            % Smooth rotation
            robot.theta = robot.theta + 0.25 * angleError;

            % Move only if aligned
            if abs(angleError) < 0.4
                step = min(0.5, dist);

                robot.x = robot.x + step*cos(robot.theta);
                robot.y = robot.y + step*sin(robot.theta);

                totalDistance = totalDistance + step;
            end
        end

        % Snap to waypoint (prevents spinning)
        if dist < 0.2
            robot.x = target(1);
            robot.y = target(2);
        end

        % Keep robot inside map
        robot.x = max(1,min(mapSize,robot.x));
        robot.y = max(1,min(mapSize,robot.y));

        %% ===== STORE TRAJECTORY =====
        trajectory = [trajectory; robot.x robot.y];

        %% ===== DRAW =====
        cla
        drawScene(map,items,goal,people,mainPerson,path,trajectory)

        % Display remaining items
        text(2,48,"Items Left: " + sum(~collected),'Color','y')

        drawRobot(robot,robotRadius,'g')
        drawLidar(robot,angles,lidarRange,map)

        title('FINAL: Human-Aware Robot with Trajectory')

        drawnow
        pause(0.05)

        %% ===== GOAL CHECK =====
        if norm([robot.x robot.y]-goal)<1
            simulateArm(robot)
            collected(idx) = true;
            break
        end

    end
end

%% ================= METRICS =================
disp("Total Distance: " + totalDistance)
disp("Total Stops: " + stopCount)
disp("Total Time: " + toc(startTime))

%% ================= FUNCTIONS =================

function drawScene(map, items, goal, people, mainPerson, path, trajectory)
imagesc(map')
colormap(gray)
hold on

% Items (red X)
plot(items(:,1),items(:,2),'rx','LineWidth',2)

% Goal (green circle)
plot(goal(1),goal(2),'go','LineWidth',2)

% Random humans (purple)
plot(people(:,1),people(:,2),'mo','LineWidth',2)

% Blocking human (red circle)
plot(mainPerson(1),mainPerson(2),'ro','LineWidth',3)

% Planned path (green)
if ~isempty(path)
    plot(path(:,1),path(:,2),'g','LineWidth',2)
end

% Actual trajectory (blue dashed)
if ~isempty(trajectory)
    plot(trajectory(:,1),trajectory(:,2),'b--','LineWidth',1.5)
end
end

function drawRobot(robot, r, color)
theta = linspace(0,2*pi,40);

% Robot body
fill(robot.x + r*cos(theta),robot.y + r*sin(theta),'b')

% Direction arrow
quiver(robot.x,robot.y,cos(robot.theta),sin(robot.theta),1,'w','LineWidth',2);

% Status LED
plot(robot.x, robot.y+1,'o','MarkerFaceColor',color,'Color',color)
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
