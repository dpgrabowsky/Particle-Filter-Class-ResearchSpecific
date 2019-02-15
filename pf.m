classdef pf<handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        numParticleTraits
        numParticles;
        particles;
    end
    
    methods
        function obj = pf(numParticles,numParticleTraits)
            %Create an empty set of particles equal to num particles, each
            %particles will have the dimensions of the number of
            %characteristcs. Ex) Initalize 1000 particles of a robot each  
            %having the characteristic of the robot x,y,theta. 
            obj.particles=zeros(numParticles,numParticleTraits);
            obj.numParticles=numParticles;
            obj.numParticleTraits=numParticleTraits;
        end
        
        function initRobot(obj,pose,maxRange)
            %initRobot Randomly initalize particles for a robot in a circle
            %around pose (x,y) where the radius of said circle is equal to r.
            %   Detailed explanation goes here
            n=obj.numParticles;
            center=pose(1:2);
            radius=maxRange;
            angles=2*pi*rand(n,1);
            r=radius*sqrt(rand(n,1));
            
            X=r.*cos(angles)+center(1);
            Y=r.*sin(angles)+center(2);
            
            %Initalize theta within 10% of given pose
            tMax=pose(3)+pose(3)*.10;
            tMin=pose(3)-pose(3)*.10;
            Theta=(tMax-tMin).*rand(n,1)+tMin;
            
            obj.particles=[X,Y,Theta];
        end
        
        function initLandmark(obj,pose,range,spread,option)
            %Initalize landmark particle in a donut shape with a given
            %range and spread around a pose.
            
            if(strcmp(option,'donut'))
                rMin=range-spread;
                rMax=range+spread;
                theta=rand(obj.numParticles,1)*(2*pi);
                r = sqrt((rMax^2-rMin^2)*rand(obj.numParticles,1)+rMin^2);
                x = pose(1) + r.*cos(theta);
                y = pose(2) + r.*sin(theta);
                obj.particles=[x y];
            elseif(strcmp(option,'circle'))
                rMin=0;
                rMax=range+spread;
                theta=rand(obj.numParticles,1)*(2*pi);
                r = sqrt((rMax^2-rMin^2)*rand(obj.numParticles,1)+rMin^2);
                x = pose(1) + r.*cos(theta);
                y = pose(2) + r.*sin(theta);
                obj.particles=[x y];
            end
        end
        
        function MotionModel(obj,u,d_t)
            %input: control(u), pose(x), time passed(d_t)
            %control(1): linear velocity
            %control (2): angular velocity
            %all angle expected to be in radians
            %all velocitys expected to be in m/s
            
            
            v=u(1);
            w=u(2);
            
            particle_pose=obj.particles;
            theta=particle_pose(:,3);
            r=v/w;
            
            
            s=sin(theta);
            c=cos(theta);
            s_th=sin(theta+w*d_t);
            c_th=cos(theta+w*d_t);
            if(w<.05)
                %basically robot is going straight
                particle_pose(:,1)=particle_pose(:,1)+(v*c_th)*d_t;
                particle_pose(:,2)=particle_pose(:,2)+(v*s_th)*d_t;
                particle_pose(:,3)=particle_pose(:,3)+(w)*d_t;
            else
                %robot is turning
                particle_pose(:,1)=particle_pose(:,1)+(-r*s)+(r*s_th);
                particle_pose(:,2)=particle_pose(:,2)+(r*c)-(r*c_th);
                particle_pose(:,3)=particle_pose(:,3)+w*d_t;
                
            end
            obj.particles=particle_pose;
        end
        
        function obj=MeasurmentModelLM(obj,measuredRange,robotPose)
            x_lm=obj.particles(:,1);
            y_lm=obj.particles(:,2);
            x_r=robotPose(1);
            y_r=robotPose(2);
            expectedRanges=sqrt((x_r-x_lm).^2+(y_r-y_lm).^2); 
            
            weights=[abs(measuredRange-expectedRanges),(1:obj.numParticles)'];
            weightsSorted=sortrows(weights,1);
            weightsNormalized=(weightsSorted(:,1)-min(weightsSorted(:,1)))/(max(weightsSorted(:,1))-min(weightsSorted(:,1)));
            weightsNormalized(:,2)=weightsSorted(:,2);
            weightsNormalized(:,1)=1-weightsNormalized(:,1);
            weightsNormalized=flipud(weightsNormalized);
            
            weightsNormalized(:,1)=cumsum(weightsNormalized(:,1));
            
            particles_hat=zeros(obj.numParticles,2);
            for(ii=1:obj.numParticles)
                val=min(weightsNormalized(:,1)) + (max(weightsNormalized(:,1))-min(weightsNormalized(:,1))).*rand;
                idx=find((weightsNormalized(:,1)>=val),1,'first');
                particles_hat(ii,:)=obj.particles(weightsNormalized(idx,2),:)+normrnd(0,0.05,[1 2]);
            end
            obj.particles=particles_hat;
            
            
        end
        

        function obj=MeasurmentModelRbot(obj,measuredRange,landmarkPos)
            %identical to landmark measurment model, just changed the names
            %around for clarity.
            x_r=repmat(obj.particles(:,1),1,size(landmarkPos,1));
            y_r=repmat(obj.particles(:,2),1,size(landmarkPos,1));
            %x_lm=landmarkPos(1);
            %y_lm=landmarkPos(2);
            x_lm=landmarkPos(:,1);
            y_lm=landmarkPos(:,2);
            expectedRanges=sqrt((x_lm'-x_r).^2+(y_lm'-y_r).^2);
            
            
            if(size(expectedRanges,2)>1)
                weights=[sum(abs(measuredRange-expectedRanges),2),(1:obj.numParticles)'];
            else
                
                weights=[abs(measuredRange-expectedRanges),(1:obj.numParticles)'];
            end
            
            
          %  weights=[abs(measuredRange-expectedRanges),(1:obj.numParticles)'];
            weightsSorted=sortrows(weights,1);
            weightsNormalized=(weightsSorted(:,1)-min(weightsSorted(:,1)))/(max(weightsSorted(:,1))-min(weightsSorted(:,1)));
            weightsNormalized(:,2)=weightsSorted(:,2);
            weightsNormalized(:,1)=1-weightsNormalized(:,1);
            weightsNormalized=flipud(weightsNormalized);
            
            weightsNormalized(:,1)=cumsum(weightsNormalized(:,1));
            
            particles_hat=zeros(obj.numParticles,3);
            for(ii=1:obj.numParticles)
                val=min(weightsNormalized(:,1)) + (max(weightsNormalized(:,1))-min(weightsNormalized(:,1))).*rand;
                idx=find((weightsNormalized(:,1)>=val),1,'first');
                particles_hat(ii,:)=obj.particles(weightsNormalized(idx,2),:)+normrnd(0,0.05,[1 3]);
            end
            obj.particles=particles_hat;
            
            
        end
        
        
        function obj=MeasurmentModelFull(obj,measuredRange,lmIDX)
            %identical to landmark measurment model, just changed the names
            %around for clarity.
            x_r=obj.particles(:,1);
            y_r=obj.particles(:,2);
            x_lm=obj.particles(:,2+lmIDX*2);
            y_lm=obj.particles(:,3+lmIDX*2);
            expectedRanges=sqrt((x_lm-x_r).^2+(y_lm-y_r).^2);
            
            
            if(size(expectedRanges,2)>1)
                weights=[sum(abs(measuredRange-expectedRanges),2),(1:obj.numParticles)'];
            else
                
                weights=[abs(measuredRange-expectedRanges),(1:obj.numParticles)'];
            end
            weightsSorted=sortrows(weights,1);
            weightsNormalized=(weightsSorted(:,1)-min(weightsSorted(:,1)))/(max(weightsSorted(:,1))-min(weightsSorted(:,1)));
            weightsNormalized(:,2)=weightsSorted(:,2);
            weightsNormalized(:,1)=1-weightsNormalized(:,1);
            weightsNormalized=flipud(weightsNormalized);
            
            weightsNormalized(:,1)=cumsum(weightsNormalized(:,1));
            
            particles_hat=zeros(obj.numParticles,length(obj.particles(1,:)));
            for(ii=1:obj.numParticles)
                val=min(weightsNormalized(:,1)) + (max(weightsNormalized(:,1))-min(weightsNormalized(:,1))).*rand;
                idx=find((weightsNormalized(:,1)>=val),1,'first');
                particles_hat(ii,:)=obj.particles(weightsNormalized(idx,2),:)+normrnd(0,0.05,[1 length(particles_hat(ii,:))]);
            end
            obj.particles=particles_hat;
            
            
        end
        
        function initLandmarkFull(obj,startPoint,idx,range,spread,option)
            %Initalize landmark particle in a donut shape with a given
            %range and spread around a pose.
            
            if(strcmp(option,'donut'))
                rMin=range-spread;
                rMax=range+spread;
                theta=rand(obj.numParticles,1)*(2*pi);
                r = sqrt((rMax^2-rMin^2)*rand(obj.numParticles,1)+rMin^2);
                x = startPoint(1) + r.*cos(theta);
                y = startPoint(2) + r.*sin(theta);
                obj.particles(:,2+idx*2:3+idx*2)=[x y];
            elseif(strcmp(option,'circle'))
                rMin=0;
                rMax=range+spread;
                theta=rand(obj.numParticles,1)*(2*pi);
                r = sqrt((rMax^2-rMin^2)*rand(obj.numParticles,1)+rMin^2);
                x = startPoint(1) + r.*cos(theta);
                y = startPoint(2) + r.*sin(theta);
                obj.particles(:,2+idx*2:3+idx*2)=[x y];
            end
        end
        
        
        
    end
end

