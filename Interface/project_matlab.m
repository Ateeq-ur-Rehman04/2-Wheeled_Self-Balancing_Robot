% 1. Setup Serial Connection
clear; clc;
if ~isempty(serialportfind)
    fclose(serialportfind); delete(serialportfind); % Clean up old ports
end

arduinoObj = serialport("COM3", 115200); % Select your port
configureTerminator(arduinoObj, "CR/LF"); % Matches Arduino println
flush(arduinoObj);

% 2. Setup Live Plotting
figure('Name', 'Robot Dashboard', 'Color', 'white');
hAngle = animatedline('Color', 'r', 'LineWidth', 2);
hMotor = animatedline('Color', 'b', 'LineWidth', 1);
axis([0 100 -50 50]); % Adjust Y-axis limits (Angle)
grid on;
legend('Tilt Angle', 'Motor PWM');
title('Self-Balancing Robot Telemetry');
ylabel('Value'); xlabel('Time (samples)');

% 3. Data Acquisition Loop
stopLoop = false;
sampleCount = 0;

while ~stopLoop
    try
        % Read one line of data
        dataLine = readline(arduinoObj);
        
        % Parse CSV (Split by comma)
        % Format: Angle, LeftPWM, RightPWM, Distance
        dataValues = str2double(split(dataLine, ','));
        
        if length(dataValues) == 4
            angle = dataValues(1);
            pwmLeft = dataValues(2);
            
            % Update Plot
            addpoints(hAngle, sampleCount, angle);
            addpoints(hMotor, sampleCount, pwmLeft/10); % Scaled down to fit graph
            
            % Force Draw
            drawnow limitrate;
            sampleCount = sampleCount + 1;
        end
    catch
        disp('Read Error');
    end
end