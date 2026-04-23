Ra=5;
Ke=0.006;
Kt=0.006;
Gr=1/48;
rw=0.0325;
Jm=2*(10^-7);
mw=0.038;
Jw=0.5*mw*(rw)^2;
lg=0.05;
mp=0.46;
Jp=0.0030;
% ==========================================
% 1. DEFINE CONSTANTS
% ==========================================

% --- Motor Parameters ---
Ra = 5;              % Armature Resistance (Ohms)
Ke = 0.006;          % Back EMF Constant
Kt = 0.006;          % Torque Constant
Gr = 1/48;           % Gear Ratio (Note: 1/48, not 48)
b  = 0;              % Viscous friction (Assumed 0 per user request)
Jm = 2e-7;           % Motor Rotor Inertia

% --- Wheel Parameters ---
% CRITICAL FIX: rw must be radius, not diameter (65mm / 2 = 32.5mm)
rw = 0.0325;         
mw = 0.038;          % Mass of wheel
Jw = 0.5 * mw * rw^2;% Wheel Inertia calculation

% --- Robot Body (Pendulum) Parameters ---
% Values taken from your robot estimation
lg = 0.05;           % Distance to Center of Gravity
mp = 0.46;           % Mass of Robot Body
Jp = 0.0030;         % Moment of Inertia of Body

% ==========================================
% 2. BUILD THE TRANSFER FUNCTION
% Based on Equation from your Image (image_a01758.png)
% ==========================================

% --- Helper Terms (to keep the math clean) ---

% The term (Jp - mp*lg^2) appears twice in the denominator
Inertia_Common = (Jp - mp * lg^2); 

% The large bracket term for s^2
% [ Jm/Gr + Jw + mp*rw^2 + mw*rw^2 + mp*lg*rw ]
Term_s2_Bracket = (Jm/Gr) + Jw + (mp * rw^2) + (mw * rw^2) + (mp * lg * rw);

% The bracket term for s^1
% [ (Kt*Ke)/(Ra*Gr) + b/Gr ]
Term_s1_Bracket = ((Kt * Ke) / (Ra * Gr)) + (b / Gr);


% --- Numerator Construction ---
% (Kt/Ra) * (mp * lg * rw)
Num_Val = (Kt / Ra) * (mp * lg * rw);
num = [0, 0, Num_Val];


% --- Denominator Construction ---
% s^2 Coefficient: - (Jp - mp*lg^2) * [Big Inertia Bracket]
den_s2 = - Inertia_Common * Term_s2_Bracket;

% s^1 Coefficient: - (Jp - mp*lg^2) * [Electrical/Damping Bracket]
den_s1 = - Inertia_Common * Term_s1_Bracket;

% s^0 Coefficient: 0 (No constant term in your equation)
den_s0 = 0;

den = [den_s2, den_s1, den_s0];


% ==========================================
% 3. CREATE AND DISPLAY SYSTEM
% ==========================================
sys = tf(num, den);

disp('Transfer Function Generated Successfully:');
sys