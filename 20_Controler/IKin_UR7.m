%% 1. Neue Roboter-Parameter (UR7 Beispielwerte)
% Passe diese Werte an dein spezifisches UR7-Modell an
L1 = 0.350; 
L2 = 0.300; % Beispiel Länge 2
W1 = 0.150;
W2 = 0.100;
H1 = 0.1625;
H2 = 0.090;

% Home-Position Matrix M (Endeffektor bei theta=0)
M = [-1 0 0 L1+L2;
      0 0 1 W1+W2;
      0 1 0 H1-H2;
      0 0 0 1];

% Raum-Schraubachse (Space Screw Axes) S
S1 = [0 0 1 0 0 0]';
S2 = [0 1 0 -H1 0 0]';
S3 = [0 1 0 -H1 0 L1]';
S4 = [0 1 0 -H1 0 L1+L2]';
S5 = [0 0 -1 -W1 L1+L2 0]';
S6 = [0 1 0 H2-H1 0 L1+L2]';
S = [S1 S2 S3 S4 S5 S6];

% Körper-Schraubachse (Body Screw Axes) B
B = zeros(6,6);
for i = 1:6
    B(:,i) = ECE569_Adjoint(inv(M)) * S(:,i);
end


%% 2. Inverse Kinematik für die gewünschte Position
% Angenommen: 'states' enthält [x, vx, y, vy, ...] aus deiner Labyrinth-Simulation
% Wir nutzen hier nur x(1) und states(3) für die Position

N = length(t);
thetaAll = zeros(6, N);
theta0 = [-1.6, -1.4, -1.8, -3.0, -0.9, 0]'; % Dein Start-Guess (nahe theta0)
thetaAll(:,1) = theta0;

% Start-Transformation berechnen
T0 = ECE569_FKinBody(M, B, theta0); 

for i = 2:N
    % Gewünschte Position aus der Simulation (relativ zu T0)
    pd = [states(i,1); states(i,3); 0]; % x und y aus Labyrinth
    Rd = eye(3); 
    Td_rel = [Rd, pd; 0 0 0 1];
    
    Tsd_target = T0 * Td_rel; % Ziel-Transformation im Raum
    
    % IK Schritt mit dem vorherigen Ergebnis als Startwert
    initialguess = thetaAll(:, i-1);
    [thetaSol, success] = IKinBody(B, M, Tsd_target, initialguess, 1e-6, 1e-6);
    
    if ~success
        warning('IK konnte an Schritt %d nicht gelöst werden.', i);
        thetaAll(:, i) = thetaAll(:, i-1); % Behalte alten Wert bei Fehler
    else
        thetaAll(:, i) = thetaSol;
    end
end

