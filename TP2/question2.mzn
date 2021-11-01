include "globals.mzn"; 

int: n = 5;
% the given results
array[1..4] of int: res = [1,2,5,7];

% number of points for each player
array[1..n] of var 0..3*n: x;

% (i, j) is the result of player i against player j
array[1..n, 1..n] of var {0,1,3}: matches;

% last player has our unknown score
% assign scores to the first 4 players
constraint forall(i in 1..n-1)(
  x[i] = res[i]);

% players cannot play themselves
constraint forall(i in 1..n)(
  matches[i,i] = 0);

% all the and rows need to add up to the final scores
% we have only 1 matches matrix since players only play each other once
constraint forall(i in 1..n)(
  sum(j in 1..n)(matches[i,j]) = x[i]);

% a win implies a loss for the other player and a tie implies a tie
% this means that if matches[1,2] = 3 then matches[2,1] = 0 
% and that if matches[1,3] = 1 then matches[3,1] = 1
% to avoid contradictions, only check the top right diagonal (j > i)
constraint forall(i in 1..n)(
  forall(j in 1..n)(
    if j > i then
      if matches[i,j] == 1 then matches[j,i] = 1
        else if matches[i,j] == 3 then matches[j,i] = 0
          else if matches[i,j] == 0 then matches[j,i] = 3 endif endif endif endif));

solve satisfy;

output 
[
  "Scores\n"
]
++
[
  show(x)
]
++	
[
  "\n\nMatches\n\n"
]
++
["       | Opponent  | \n       | 1 2 3 4 5 |\nPlayer |-----------| Points\n"]
++
[
         if j == 1 then show(i) ++ "      | " ++ show(matches[i,j])
         elseif j == n then " " ++ show(matches[i,j]) ++ " | " ++ show(x[i]) ++ "\n" else " " ++ show(matches[i,j]) endif |
         i in 1..n, j in 1..n
]
;