clear
clc
close all
global u;
global turtlebot

rosshutdown
addpath('../turtlebot_measurment_class');
addpath('../pozyx_tag_class');


fprintf('Initalize turtlebot object');

turtlebot=turtlebot_measurment_class(...
    'ROS_MASTER_URI','http://10.0.0.131:11311',...
    'ROS_IP','10.0.0.7',...
    'SUBSCRIBER',{'/odom','/cmd_vel_mux/input/teleop'},...
    'PUBLISHER',{'/mobile_base/commands/velocity'});


fprintf('\n');


numParticles=2000;
tID='0x0000';
aID={'0x6940','0x6935','0x6937','0x6955'};
colors={'yellow','red','green','blue'};


%Init tag_obj for storing pozyx measurments
tag_obj=pozyx_tag(tID,aID,'/pozyx_device_range');


%Create and init pf filter for robot
fprintf('Initalizing robot particle filter... \n');
initPose=[0,0,0];
robotFilter=pf(numParticles,3);
robotFilter.initRobot(initPose,1)
fprintf('Robot particle filter initalized\n\n');


fprintf('Initalizing landmark particle filtes...\n');
%Create filters for landmarks
landmarkFilters={};
for(ii=1:length(aID))
   landmarkFilters{ii}=pf(numParticles,2);
end

%Wait till a measurment from each anchor is gathered before initing
%landmarkFilters
fprintf('Waiting for inital range measurments\n');
initRanges=zeros(1,length(aID));
while~(sum(initRanges~=0)==length(aID))
    for(ii=1:length(aID))
        if(~isempty(cell2mat(tag_obj.youngDistanceMeasure(ii))))
            temp=tag_obj.youngDistanceMeasure{ii};
            initRanges(ii)=temp(1,end);
        end
    end
    pause(0.05)
end
fprintf('Inital range measurments receieved\n');
for(ii=1:length(aID))
    landmarkFilters{ii}.initLandmark(initPose(1:2),initRanges(ii),1,'donut');
end
fprintf('Landmarks initalized\n\n');

%% Start velocity pub timer

% fprintf('Starting velocity publisher timer...\n')
% t2=timer('StartDelay',.1,'ExecutionMode','fixedRate','Period',0.2,'TimerFcn',@velPublisher);
% start(t2);
% fprintf('Velocity publisher timer started \n\n');

tic;
t1=toc;
robotPose=[0,0,0];
u=[0.1,0.0];
u2=turtlebot.receive('/cmd_vel_mux/input/teleop');
u2=[u2.Linear.X u2.Angular.Z];


h=figure(1);

e={0 0 0 0};
for(ii=1:700)

      
   u1=turtlebot.receive('/cmd_vel_mux/input/teleop');
   u1=[u1.Linear.X u1.Angular.Z];   
    
    %% Move the robot
    if(ii==75)
       u=[0.0 0.0]; 
    elseif(ii==100)
        u=[0.1,0.3];
    elseif(ii==140)
        u=[0.1 0.0];
    elseif(ii==320)
        u=[0 0];  
    end
    
    u=u1;
    
    %% Run the measurment model for the robot
    t2=toc;
    dt=t2-t1;
    t1=t2;
    robotFilter.MotionModel(u,dt);
    robotPose=[mean(robotFilter.particles(:,1)),mean(robotFilter.particles(:,2)),mean(robotFilter.particles(:,3))];
    
    %% For each landmark, check to see if there is a recent measurment avaiable
    ranges=cell(1,length(aID));
    tag_obj.RemoveOldMeasurments(2);
    for(jj=1:length(aID))
        if(~isempty(tag_obj.youngDistanceMeasure{jj}))
           ranges{jj}=tag_obj.youngDistanceMeasure{jj}(1,end); 
        end
    end
    
    %% Run the measurment model for each landmark that had a measurment
    for(jj=1:length(aID))
        if(~isempty(ranges{jj}))
            landmarkFilters{jj}.MeasurmentModel(ranges{jj},robotPose(1:2));
        end
    end


    %% Plotting
    h=sfigure(h);
    
    xoff=5.8929;
    yoff=3.6576;
    
    scatter(robotFilter.particles(:,1)+xoff,robotFilter.particles(:,2)+yoff);
    hold on
    
    for(jj=1:length(aID))
        scatter(landmarkFilters{jj}.particles(:,1)+xoff,landmarkFilters{jj}.particles(:,2)+yoff,20,colors{jj});
    end
    hold off
    xlim([-3.5,10.5]);
    ylim([-3.5 10.5]);
    
    
    %% Data Validation
    anchors.names={...
        '0x6940';...
        '0x6935';...
        '0x6937';...
        '0x6955'};
    anchors.coords=[...
        6.71,0.00;...
        6.71,3.66;...
        0.00,0.00;...
        0.00,3.66];
    for(jj=1:length(aID))
       actual=anchors.coords(jj,:);
       estimated=[mean(landmarkFilters{jj}.particles(:,1)+xoff),mean(landmarkFilters{jj}.particles(:,2)+yoff)];
       error=actual-estimated;
       error=sqrt(error(1)^2+error(2)^2);
       e{jj}=[e{jj};error];
    end
    
    %% END
    pause(.005);
end


function velPublisher(src,event)
global u;
global turtlebot;
    msg=rosmessage('geometry_msgs/Twist');
    msg.Linear.X=u(1);
    msg.Angular.Z=u(2);
    turtlebot.send('/mobile_base/commands/velocity',msg);
end



function h = sfigure(h)
% SFIGURE  Create figure window (minus annoying focus-theft).
%
% Usage is identical to figure.
%
% Daniel Eaton, 2005
%
% See also figure
if nargin>=1 
	if ishandle(h)
		set(0, 'CurrentFigure', h);
	else
		h = figure(h);
	end
else
	h = figure;
end
end