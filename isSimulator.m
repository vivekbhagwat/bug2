function b = isSimulator(obj)
b = strcmp(class(obj),'CreateRobot');
end