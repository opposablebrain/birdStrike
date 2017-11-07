function totalRange = senseAvoidMinRange(turnN,minSep,cruiseV,onV,setPlot)

global turnRadius turnRate oncomingSpeed minSeparation

cruiseSpeed = cruiseV; %vahana cruise speed [m/s]
oncomingSpeed = onV; %maximum speed in our airspace (250 kts) [m/s]
turnLoadFactor = turnN; %maximum turn load factor [g's]
minSeparation = minSep; %minimum separation distance during avoid maneuver [m]
senseTime = 2.5; %time to sense and plan [s]
g=9.81; %gravity [m/s^2]

%Avoidance maneuver
turnRate = g*sqrt(turnLoadFactor^2-1)/cruiseSpeed; %turn rate [rad/s]
turnRadius = cruiseSpeed^2 / g / sqrt(turnLoadFactor^2-1); %turn radius [m]
options = optimoptions(@fsolve,'Display','off');
turnTime = fsolve(@(t)thetaError(t),.001,options); %time to avoid [s]);
[~,theta]=thetaError(turnTime);

%Range calculation
turnDistance = turnRadius*sin(turnRate*turnTime)+oncomingSpeed*turnTime; %distance traveled directionally [m]
senseDistance=senseTime*(cruiseSpeed+oncomingSpeed); %distance traveled during sensing and planning [m]
separationDistance=minSeparation*sin(theta);
totalRange = senseDistance+turnDistance+separationDistance; %total sensing range requirement [m]

if setPlot
    N=100;
    t=linspace(0,turnTime,N);
    x=turnRadius*sin(turnRate*t)+senseTime*cruiseSpeed;
    y=turnRadius*(1-cos(turnRate*t));
    x0=x(end)+separationDistance;
    phi=linspace(0,2*pi,N);
    
    figure(1); clf; hold on
    plot([0 senseTime]*cruiseSpeed,[0 0],'b');
    plot(x,y,'r')
    plot(x0,0,'k*')
    plot(x0+minSeparation*cos(phi),minSeparation*sin(phi),'k')
    plot(x0+turnTime*oncomingSpeed+[0 senseTime*oncomingSpeed],[0 0],'b--')
    plot(x0+[0 turnTime*oncomingSpeed],[0 0],'r--')
    legend('Sense & Plan','Avoid','Object','Minimum separation')
    axis equal
    
    xlabel('Directional distance traveled [m]')
    ylabel('Lateral distance traveled [m]')
    title({[num2str(round(100*turnLoadFactor)/100) 'g Avoidance Maneuver'],...
        ['Cruise speed = ' num2str(round(cruiseSpeed)) ' m/s. Oncoming object speed = ' num2str(round(oncomingSpeed)) ' m/s.'],...
        ['Minimum separation = ' num2str(round(minSeparation)) ' m. Sensing range = ' num2str(round(totalRange)) ' m.']})
end

function [err,theta]=thetaError(t)
global turnRadius turnRate oncomingSpeed minSeparation

R=turnRadius;
p=turnRate;
d=minSeparation;
V=oncomingSpeed;

y=R*(1-cos(p*t));
tanTheta=sqrt(d^2-y^2)/y;
dydx = R*sin(p*t)*p/(R*cos(p*t)*p+V);
err = (tanTheta-dydx)^2;
theta=atan(tanTheta);