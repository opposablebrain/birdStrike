load birdSizeData.mat

cameraViewAngle=30*pi/180; %Camera viewing angle [rad]
cameraResolution = [4000 3000]; %Camera pixels [width height]
minObjectResolution = 30; %Minimum window size to see an object
minObjectAngle = cameraViewAngle*minObjectResolution/cameraResolution(1);

frameRate = 10; %processable frame rate [1/s]
framesToDetect = 3; %number of frames to detect
timeToProcess = framesToDetect/frameRate; %time to process detection
timeToAct = 1; %time for (path plan + MCA + initial maneuver) [s]

cruiseSpeed = 50; %m/s
testBirdMass = 1.81; %mass of bird used in strike test [kg]
testEnergy = 0.5*testBirdMass*cruiseSpeed^2;
wingSpan = 6.2; %Alpha wingspan [m]

turnLoadFactor = 2; %g's pulled in turn
g=9.81; %gravity
turnRate = g*sqrt(turnLoadFactor^2-1)/cruiseSpeed; %turn rate [rad/s]
turnRadius = cruiseSpeed^2 / g / sqrt(turnLoadFactor^2-1); %turn radius
avoidanceFactor = 5; %multiple of bird wingspan to avoid by

taxons = unique(birdSizeData.taxon); %bird taxons in dataset
massErrorRatio = 0.1; %assumed percent error on mass

for i=1:height(birdSizeData)
    span(i) = birdSizeData.meanWingspan(i); %bird wingspan [m]
    distance(i) = span(i)/minObjectAngle; 
    velocity(i) = birdSizeData.meanEquivalentSpeed(i);
    velocityError = birdSizeData.stddev(i);
    mass(i) = birdSizeData.totalBodyMass(i);
    energy(i) = 0.5*mass(i)*(velocity(i)+cruiseSpeed)^2;
    energyError(i) = sqrt(massErrorRatio^2+4*(velocityError/velocity(i))^2)*energy(i);
    taxon(i) = find(contains(taxons,birdSizeData.taxon(i)));
    
    avoidanceSpacing = avoidanceFactor*span(i)+(wingSpan+span(i))/2; %distance to avoid by
    phi = acos(1-avoidanceSpacing/turnRadius); %turn angle to avoid
    avoidanceTime(i) = phi/turnRate; %Time to avoid
    avoidanceDistance(i) = turnRadius*sin(phi); %Horizontal distance to avoid
    distanceToImpact(i) = avoidanceTime(i)*(cruiseSpeed+velocity(i)); %fwd distance traveled 
    excessTimeToAvoid(i) = (distance(i)-distanceToImpact(i))/(cruiseSpeed+velocity(i))-timeToProcess-timeToAct; %difference in avoidance time to detection distance
end

figure(1); clf; hold on
for i=1:length(taxons)
    plot(excessTimeToAvoid(taxon==i),energy(taxon==i)/1000,'.','markersize',20);
end
xrange = xlim; yrange=ylim;
area(xrange,testEnergy*[1 1]/1000,'facecolor','g'); alpha(.1)
area([0 xrange(2)],yrange(2)*[1 1],'facecolor','b'); alpha(.1)
xlabel('Excess time to avoid [s]')
ylabel('Total impact energy [kJ]')
legend(taxons{:},'Impactable','Avoidable','location','northwest')
title({'Bird Avoidance vs Impact Capability';['Vc=' num2str(round(cruiseSpeed)) 'm/s, turn=', num2str(round(turnLoadFactor*10)/10) 'g, clearance=', num2str(round(avoidanceFactor)) ' wingspans']});
%%
% 
% figure(1); clf; hold on
% for i=1:length(taxons)
%     plot(span(taxon==i),velocity(taxon==i),'.','markersize',20);
% end
% xlabel('Span [m]')
% ylabel('Speed [m/s]')
% legend(taxons{:},'location','northwest')
