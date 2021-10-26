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

% place all the lights from the garden in our decision variable
constraint forall(i,j in 1..n where garden[i,j] == L)(
  x[i,j] = L);
  
% make sure we do not place any other lamps
constraint forall(i,j in 1..n where garden[i,j] != L)(
  x[i,j] != L);

% since P = 2, we double the value of rows and cols
% make sure to only set Plants (P) where there are no Lamps (L)
% make sure the sum of plants in rows and columns corresponds to given rows/cols
constraint forall(i in 1..n)(sum(j in 1..n where garden[i,j] != L )(x[i,j]) = rows[i]*2);
constraint forall(j in 1..n)(sum(i in 1..n where garden[i,j] != L)(x[i,j]) = cols[j]*2);

% Make sure that all lamps are directly touching at least one plant
constraint forall(i,j in 1..n where garden[i,j] == L)(
  (if i != n then x[i+1,j] == P else false endif) \/
  (if i != 1 then x[i-1,j] == P else false endif) \/
  (if j != n then x[i,j+1] == P else false endif) \/
  (if j != 1 then x[i,j-1] == P else false endif));

% plants must not touch eachother directly or diagonnaly
constraint forall(i,j in 1..n where x[i,j] == P)(
  (if i != n then x[i+1,j] != P  endif) /\
  (if i != 1 then x[i-1,j] != P endif) /\
  (if j != n then x[i,j+1] != P endif) /\
  (if j != 1 then x[i,j-1] != P endif) /\
  (if (i !=n /\ j !=n) then x[i+1,j+1] != P endif) /\
  (if (i !=n /\ j !=1) then x[i+1,j-1] != P endif) /\
  (if (i !=1 /\ j !=n) then x[i-1,j+1] != P endif) /\
  (if (i !=1 /\ j !=1) then x[i-1,j-1] != P endif));


% make sure that all the plants are directly touching at least one lamp
constraint forall(i,j in 1..n where x[i,j] == P)(
  (if i != n then x[i+1,j] == L else false endif) \/
  (if i != 1 then x[i-1,j] == L else false endif) \/
  (if j != n then x[i,j+1] == L else false endif) \/
  (if j != 1 then x[i,j-1] == L else false endif));

solve satisfy;

output 
[
 if j = 1 then "\n" ++ show(rows[i]) ++ " " else " " endif ++
   if fix(x[i,j]) = L then "L" elseif fix(x[i,j]) = P then "P" else "_" endif
| i,j in 1..n
] ++ ["\n  "] ++ [show(cols[i]) ++ " " | i in 1..n];
