include "globals.mzn"; 
include "regular.mzn";

int: n; % num rows and columns

int: L = 1; % Lamp
int: P = 2; % Plant

% 1 for a lamp, 2 for a plant and 0 for nothing 
array[1..n,1..n] of 0..2: garden;

% number of plants by row/col
array[1..n] of int: rows;
array[1..n] of int: cols;

% decision variables
array[1..n,1..n] of var 0..P: x;

predicate count(array[$X] of var int: x, var int: y, var int: c) =
    count_eq(array1d(x), y, c);

% place the lamps in the garden
constraint forall(i in 1..n)(
  forall(j in 1..n)(
    x[i,j] = garden[i,j]));




% the sum of lines and columns must respect the technical limitations
% constraint forall(i in 1..n)(sum(j in 1..n)(x[i,j]) = rows[i]);

% % the sum of lines and columns must respect the technical limitations
% constraint forall(j in 1..n)(sum(i in 1..n)(x[i,j]) = cols[j]);

output 
[
 if j = 1 then "\n" ++ show(rows[i]) ++ " " else " " endif ++
   if fix(x[i,j]) = L then "L" elseif fix(x[i,j]) = P then "P" else "_" endif
| i,j in 1..n
] ++ ["\n  "] ++ [show(cols[i]) ++ " " | i in 1..n];
