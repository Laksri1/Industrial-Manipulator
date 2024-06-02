% Define a 4x4 matrix
EET = rand(4, 4); % or you can define it explicitly, e.g., EET = [1 2 3 4; 5 6 7 8; 9 10 11 12; 13 14 15 16];

% Check the size of EET
disp(size(EET)); % Should display [4 4]

% Access the elements in rows 1 to 3 and column 4
column_vector = EET(1:3, 4);

% Display the result
disp(column_vector);

EEp = zeros(5, 3);

disp(EEp);

EEp(1,:)= EET(1:3,4);

disp(EEp);

t = 0: 0.04: 2;

disp(length(t));