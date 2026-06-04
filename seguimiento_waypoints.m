%% Definicion del vehiculo
R  = 0.1;                 % Radio de rueda [m]
L  = 0.5;                 % Distancia entre ruedas [m]
dd = DifferentialDrive(R,L);


% --- Grafica 1 
% waypoints = [ 4, 4;   % A
%              -9, 8;   % B
%               7,-1;   % C
%              -9,-5;   % D
%              -1, 5;   % E
%              -3, 0;   % F
%               3,-5;   % G
%               0, 0];  % H

% --- Grafica 2 
% waypoints = [ 2, 5;   % A
%              -5, 3;   % B
%              -5,-2;   % C
%               2,-5;   % D
%               5, 2;   % E
%              -3, 2;   % F
%              -4,-4;   % G
%               4,-3];  % H

% --- Grafica 3 
waypoints = [-3, 4;   % A
              3, 3;   % B
              2,-3;   % C
             -1,-1;   % D
              1, 4;   % E
             -2,-4;   % F
              2,-1];  % G

%% Parametros de simulacion
sampleTime = 0.1;                              % Tiempo de muestreo [s]

desiredV = 1.0;                                % Velocidad lineal deseada [m/s]
pathLen  = sum(vecnorm(diff(waypoints),2,2));  % Longitud total de la trayectoria
Tsim     = 1.4*pathLen/desiredV;               % Tiempo total (+40% de margen)
tVec     = 0:sampleTime:Tsim;                  % Vector de tiempo

% Pose inicial: en el primer waypoint, orientada hacia el segundo
theta0   = atan2(waypoints(2,2)-waypoints(1,2), ...
                 waypoints(2,1)-waypoints(1,1));
initPose = [waypoints(1,1); waypoints(1,2); theta0];   % [x; y; theta]

pose = zeros(3,numel(tVec));                    % Matriz de poses
pose(:,1) = initPose;

%% Visualizador
viz = Visualizer2D;
viz.hasWaypoints = true;

%% Controlador Pure Pursuit
controller = controllerPurePursuit;
controller.Waypoints = waypoints;
controller.LookaheadDistance     = 0.5;        % sube si oscila, baja si corta curvas
controller.DesiredLinearVelocity = desiredV;
controller.MaxAngularVelocity    = 2.0;        % mas alto = giros mas cerrados

%% Lazo de simulacion
close all
r = rateControl(1/sampleTime);
for idx = 2:numel(tVec)
    % Pure Pursuit -> velocidades de referencia
    [vRef,wRef] = controller(pose(:,idx-1));

    % Cinematica inversa -> velocidades de rueda
    [wL,wR] = inverseKinematics(dd,vRef,wRef);

    % Cinematica directa -> velocidades del cuerpo
    [v,w] = forwardKinematics(dd,wL,wR);
    velB  = [v;0;w];                            % [vx; vy; w] en el cuerpo
    vel   = bodyToWorld(velB,pose(:,idx-1));    %

    % Integracion discreta (Euler)
    pose(:,idx) = pose(:,idx-1) + vel*sampleTime;

    % Visualizacion
    viz(pose(:,idx),waypoints)
    waitfor(r);
end
