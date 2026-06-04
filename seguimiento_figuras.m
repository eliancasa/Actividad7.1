%% Definicion del vehiculo
R  = 0.1;                 % Radio de rueda [m]
L  = 0.5;                 % Distancia entre ruedas [m]
dd = DifferentialDrive(R,L);


% =================== FIGURA 1 : PERRO===================
% waypoints = [ 5,12;  3,10;  1, 9;  1, 8;  2, 6;  3, 7;  4, 7; ...   
%               5, 6;  6, 5;  6, 4;  7, 0; ...                        
%               6, 5; 11, 6; 12, 6; ...                               
%               6, 4; 10, 6; ...                                      
%              11, 6;  8, 8;  7,12; ...                               
%               3,10;  4, 9;  4,10];                                  

% =================== FIGURA 2 : ESTRELLA  ===================
% waypoints = [ 0, 5;  2, 7;  0, 9;  2,11;  4, 9;  6,11;  8, 9;  6, 7;  8, 5; ... 
%               3, 6;  5, 6;  5, 8;  3, 8;  3, 6; ...                 
%               0, 5;  2, 5;  2, 3;  0, 3;  0, 5; ...                 
%               8, 5;  6, 5;  6, 3;  8, 3;  8, 5; ...                 
%               2, 3;  4, 5;  6, 3;  4, 1;  2, 3; ...                 
%               4, 5;  4, 1; ...                                      
%               0, 3;  2, 1;  4, 1; ...                               
%               8, 3;  6, 1;  4, 1];                                  

% =================== FIGURA 3 ===================
waypoints = [ 3, 5;  3, 3;  5, 1;  7, 1;  9, 3;  9, 5;  7, 7;  5, 7;  3, 5; ...  
              4, 9;  6,11;  8,11; 10, 9;  4, 9; ...                             
              10, 9;  6, 5; ...                                                 
              5, 6;  6, 5;  7, 5];                                             
% ---------------------------------------------------------------------------------

%% Parametros de simulacion 
sampleTime = 0.1;                              % Tiempo de muestreo [s]

desiredV = 1.0;                                % Velocidad lineal deseada [m/s]
pathLen  = sum(vecnorm(diff(waypoints),2,2));  % Longitud total del recorrido
Tsim     = 1.3*pathLen/desiredV;               % Tiempo total (+30% de margen)
tVec     = 0:sampleTime:Tsim;                  % Vector de tiempo

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
controller.LookaheadDistance     = 0.4;        
controller.DesiredLinearVelocity = desiredV;
controller.MaxAngularVelocity    = 2.5;        

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
    velB  = [v;0;w];                           
    vel   = bodyToWorld(velB,pose(:,idx-1));   

    % Integracion discreta (Euler)
    pose(:,idx) = pose(:,idx-1) + vel*sampleTime;

    % Visualizacion
    viz(pose(:,idx),waypoints)
    waitfor(r);
end
