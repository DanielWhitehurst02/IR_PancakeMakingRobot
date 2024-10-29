% delete(serialObj);
clear;
% serialportlist("available")
serialObj = serialport("/dev/ttyUSB0",9600);
configureTerminator(serialObj,"CR/LF");
flush(serialObj);
serialObj.UserData = struct("Data",[],"Count",1)

% while truecle
while true
data = readline(serialObj);
pause(0.1)
disp(data)
flush(serialObj);

end
% flush(serialObj);
% pause(1)
% print(data)
% end
% function readSineWaveData(src, ~, maxDataPoints)
% 
% % Read the ASCII data from the serialport object.
% data = readline(src);
% 
% % Convert the string data to numeric type and save it in the UserData
% % property of the serialport object.
% src.UserData.Data(end+1) = str2double(data);
% 
% % Update the Count value of the serialport object.
% src.UserData.Count = src.UserData.Count + 1;
% 
% % If over maxDataPoints points have been collected from the Arduino, switch off the
% % callbacks and plot the data, starting from the second point. 
% if src.UserData.Count > maxDataPoints
%     configureCallback(src, "off");
%     plot(src.UserData.Data(2:end));
% end
% end