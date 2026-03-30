clc
clear
close all

%% ================= MAP =================
mapSize = 50;                 % Define grid size (50x50)
map = zeros(mapSize);         % Initialize map (0 = free, 1 = obstacle)

% Create horizontal supermarket aisles (obstacles)
for y = 10:10:40
    map(y,5:45) = 1;         % Solid shelves across map
end

% Create gaps (walkways between aisles)
for y = 10:10:40
    map(y,20:25) = 0;        % Open passage
end

%% ================= ROBOT =================
robot.x = 5;                 % Initial X position
robot.y = 5;                 % Initial Y position
robot.theta = 0;             % Initial orientation (heading angle)
robotRadius = 0.7;           % Robot size (for visualization)

%% ================= METRICS =================
totalDistance = 0;           % Tracks total distance traveled
stopCount = 0;               % Number of times robot stops/replans
startTime = tic;             % Start timer
lastReplanTime = 0;          % Prevent excessive replanning

%% ================= TRAJECTORY =================
trajectory = [];             % Stores robot path history

%% ================= ITEMS =================
% Predefined item locations in store
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

collected = false(size(items,1),1);   % Boolean array for collected items

%% ================= CHECKOUT =================
checkout = [45 5];          % Final destination after shopping

%% ================= HUMANS =================
numPeople = 5;

% Random initial positions inside store
people = rand(numPeople,2)*30 + 10;

% Random velocity vectors (slow movement)
vel = randn(numPeople,2)*0.05;

%% ================= ORDER =================
% Compute optimal visiting order using nearest-neighbor heuristic
order = nearestNeighbour([robot.x robot.y],items);

%% ================= LIDAR =================
angles = linspace(-2*pi/3, 2*pi/3, 60); % Field of view (~120 degrees)
lidarRange = 10;                        % Sensor range

%% ================= FIGURE =================
figure
axis([1 mapSize 1 mapSize])
axis square
set(gca,'YDir','normal')
set(gcf,'Color','k')                   % Black background

%% ================= SHOPPING LOOP =================
for i = 1:length(order)

    idx = order(i);

    % Skip if item already collected
    if collected(idx), continue, end

    % Find reachable point near item (avoid wall)
    goal = getReachableGoal(map, items(idx,:));

    % Plan path using A*
    path = astar(map,[robot.x robot.y],goal);

    if isempty(path), continue, end

    k = 1;   % Path index

    while k <= size(path,1)

        %% ===== HUMAN MOTION =====
        for p = 1:numPeople

            % Add random acceleration (natural movement)
            vel(p,:) = vel(p,:) + 0.02*randn(1,2);

            % Limit max speed
            speed = norm(vel(p,:));
            maxSpeed = 0.15;
            if speed > maxSpeed
                vel(p,:) = vel(p,:) / speed * maxSpeed;
            end

            % Update position
            people(p,:) = people(p,:) + vel(p,:);

            % Bounce off boundaries
            for d = 1:2
                if people(p,d) < 2 || people(p,d) > mapSize-2
                    vel(p,d) = -vel(p,d);
                end
            end

            % Clamp position inside map
            people(p,:) = max(2,min(mapSize-2,people(p,:)));
        end

        %% ===== RANDOM HUMAN BLOCKING =====
        % Occasionally force a human into robot path (simulate real crowd)
        if rand < 0.03 && k < size(path,1)
            people(1,:) = path(min(k+3,size(path,1)),:);
        end

        target = path(k,:);

        % Move to next waypoint if close enough
        if norm([robot.x robot.y] - target) < 0.4
            k = k + 1;
            continue
        end

        %% ===== OBSTACLE DETECTION =====
        isBlocked = lidarDetectHumans(robot,people,angles,lidarRange);

        %% ===== REPLANNING =====
        if isBlocked && toc(startTime) - lastReplanTime > 1

            stopCount = stopCount + 1;
            lastReplanTime = toc(startTime);

            tempMap = map;

            % Convert humans into temporary obstacles
            for h = 1:size(people,1)
                hx = round(people(h,1));
                hy = round(people(h,2));

                % Inflate obstacle region (safety buffer)
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
                k = min(3,size(path,1)); % Skip jitter
            else
                continue
            end
        end

        %% ===== ROBOT MOTION =====
        dx = target(1) - robot.x;
        dy = target(2) - robot.y;
        dist = sqrt(dx^2 + dy^2);

        if dist > 0.01

            % Compute heading
            desiredTheta = atan2(dy,dx);
            angleError = wrapToPi(desiredTheta - robot.theta);

            % Smooth rotation (prevents oscillation)
            robot.theta = robot.theta + 0.15 * angleError;

            % Move forward only when facing target
            if abs(angleError) < 0.6

                step = min(0.5, dist);

                % Predict next position
                nextX = robot.x + step*cos(robot.theta);
                nextY = robot.y + step*sin(robot.theta);

                ix = round(nextX);
                iy = round(nextY);

                % Check collision with map
                if ix>=1 && iy>=1 && ix<=mapSize && iy<=mapSize
                    if map(ix,iy) == 0
                        robot.x = nextX;
                        robot.y = nextY;
                        totalDistance = totalDistance + step;
                    else
                        % Replan if hitting wall
                        path = astar(map,[robot.x robot.y],goal);
                        k = 1;
                        continue
                    end
                end
            end
        end

        %% ===== STORE TRAJECTORY =====
        trajectory = [trajectory; robot.x robot.y];

        %% ===== VISUALIZATION =====
        cla
        drawScene(map,items,goal,people,path,trajectory,collected,checkout)

        text(2,48,"Shopping... Items Left: " + sum(~collected),'Color','y')

        drawRobot(robot,robotRadius)
        drawLidar(robot,angles,lidarRange,map)

        title('🛒 Shopping Mode')

        drawnow
        pause(0.05)

        %% ===== ITEM COLLECTION =====
        if norm([robot.x robot.y]-goal)<1
            collected(idx) = true;
            break
        end
    end
end

%% ================= CHECKOUT PHASE =================
disp("Going to checkout...")

goal = checkout;
path = astar(map,[robot.x robot.y],goal);

% (Same navigation logic continues here...)

%% ================= FUNCTIONS =================

% ===== HUMAN DETECTION =====
function isBlocked = lidarDetectHumans(robot, people, angles, range)
isBlocked = false;

% Simple distance-based detection (can upgrade to ray-based later)
for p = 1:size(people,1)
    if norm([robot.x robot.y] - people(p,:)) < 2
        isBlocked = true;
        return
    end
end
end

% ===== DRAW ENVIRONMENT =====
function drawScene(map, items, goal, people, path, trajectory, collected, checkout)

imagesc(map'); colormap(gray); hold on

% Draw checkout area
rectangle('Position',[checkout(1)-3 checkout(2)-2 6 4],'FaceColor',[1 0.3 0.3])
text(checkout(1)-2,checkout(2),'CHECKOUT','Color','w')

% Draw items (only if not collected)
for i=1:size(items,1)
    if ~collected(i)
        rectangle('Position',[items(i,1)-0.5 items(i,2)-0.5 1 1],...
            'FaceColor',[0.9 0.7 0.2],'EdgeColor','none')
    end
end

% Draw humans
plot(people(:,1),people(:,2),'ro','MarkerFaceColor','r','MarkerSize',8)

% Draw planned path
if ~isempty(path), plot(path(:,1),path(:,2),'g','LineWidth',2), end

% Draw actual trajectory
if ~isempty(trajectory), plot(trajectory(:,1),trajectory(:,2),'c','LineWidth',2), end
end

% ===== DRAW ROBOT =====
function drawRobot(robot, r)
theta = linspace(0,2*pi,40);
fill(robot.x + r*cos(theta),robot.y + r*sin(theta),[0 0.8 0])
quiver(robot.x,robot.y,cos(robot.theta),sin(robot.theta),1,'Color','w','LineWidth',2)
end

% ===== DRAW LIDAR =====
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
