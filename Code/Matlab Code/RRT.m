%Code to Find sthe Optimal Path Using RRT
ss = stateSpaceSE2;
sv = validatorOccupancyMap(ss);
sv.Map = map1;
sv.ValidationDistance = 0.5;
ss.StateBounds = [map1.XWorldLimits; map1.YWorldLimits; [-pi pi]];
planner = plannerRRTStar(ss,sv);
planner.ContinueAfterGoalReached = true;
planner.MaxIterations = 3000;
planner.MaxConnectionDistance = 2.5;
start = [3,3, 0];
for i = 1:3
goal = [poseMat(i,1),poseMat(i,2), 0]; %Getting coordinates from robot
rng(100, 'twister') 
[pthObj, solnInfo] = plan(planner,start,goal);
map1.show;
hold on;
scatter(start(1),start(2),'filled')
scatter(goal(1),goal(2),'filled')
plot(pthObj.States(:,1),pthObj.States(:,2),'b-','LineWidth',2); % draw path
title('Optimal Route to Obejct'+ i)
legend('Start Point','Object Postion','Path')
end