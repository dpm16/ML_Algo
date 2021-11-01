include "globals.mzn";

int: n_nurse; % Number of nurses
int: n_week; % Number of weeks in our horizon
int: n_day; % Number of days in our horizon (7 * n_week). Monday is the first day of the week
int: n_work_period; % Number of work period. It equals 4 (MATIN, SOIREE, NUIT, REPOS)

% Indexing the work periods
int: MATIN = 1;
int: SOIREE = 2;
int: NUIT = 3;
int: REPOS = 4;

int: dur_work_period; % Duration of the work periods (6h)

array[1..n_day,1..n_work_period] of int: demande_min; % Number of nurses required for each day and each work period (= 0 for REPOS)
array[1..n_nurse,1..n_day,1..n_work_period] of int: preference; % Work preference for each nurse

% Decision variables
array[1..n_nurse,1..n_day,1..n_work_period] of var 0..1: x; % work period [MATIN, SOIREE, NUIT, REPOS]
var int: sum_preference;


%%%SOLVER CONFIGURATION USED COIN-BC%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% FIRST PART : CONSTRAINTS %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% 0. (ADDED CONSTRAINT)
% allocate a day off to unconstrained time slot
% this constraint is important to remove NONE results and make sure the days-off constraints are respected
% testing shows that this constraint did not affect the scores
constraint forall(nurse in 1..n_nurse)(
  forall(day in 1..n_day)(
    if (sum(work_period in 1..n_work_period)(x[nurse,day,work_period]) == 0) then x[nurse,day,4] = 1 endif
  )
);

%3.
% fulfill the minimum ammount of nurses required
% do not consider repos as needing to fulfill a nurse quota
constraint forall(day in 1..n_day)(
  forall(work_period in 1..(n_work_period-1))(
    (sum(nurse in 1..n_nurse)(x[nurse,day,work_period]) = demande_min[day,work_period])
    )
);

% 4.
% make sure a nurse cannot work two shifts in a single day
% to do this we set the sum of work periods per nurse per day to be less than 2
% so each nurse either works 1 shift or no shifts that day
constraint forall(nurse in 1..n_nurse)(
  forall(day in 1..n_day)(
    (sum(work_period in 1..n_work_period)(x[nurse,day,work_period]) < 2)
    ) 
  );

%5. & 6.
% a nurse cannot work more than 36 hours a week and a nurse must work at least 4 days a week
% since the length of shifts are all 6 hours and there's 1 shift a day these constraints corresponds to :
% "A nurse must have between 1 and 3 days off a week"
constraint forall(nurse in 1..n_nurse)(
  forall(week in 1..n_week)(
    (sum(day in (7*(week-1)+1)..7*week)(x[nurse,day,4]) > 0) /\
    (sum(day in (7*(week-1)+1)..7*week)(x[nurse,day,4]) < 4)
  )
);

%7.
% a nurse who works nights must have the next day off
constraint forall(nurse in 1..n_nurse)(
  forall(day in 1..(n_day-1))(
    if x[nurse,day,3] == 1 then x[nurse,day+1,4] = 1 endif
  )
);

%8.
% a nurse cannot have an isolated day of work
% to do this we look for work days and see if theres a day of rest before and after
constraint forall(nurse in 1..n_nurse)(
  forall(day in 2..(n_day-1))(
    if (x[nurse,day,4]) != 1 then (x[nurse,day-1,4] + x[nurse,day+1,4] < 2) endif
  )
);

%9.
% a nurse cannot work more than 3 days straight
% to fulfill this constraint we move a 4 day window and
% set the need of at least 1 day in that window be a rest day
constraint forall(nurse in 1..n_nurse)(
  forall(day in 1..(n_day-3))(
    (sum(window in day..(day+3))(x[nurse,window,4]) > 0)
    )
);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SECOND PART : MAXIMIZE PREFERENCES %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% sum the preference coefficient for each shift of each nurse
sum_preference = sum(nurse in 1..n_nurse,day in 1..n_day,work_period in 1..n_work_period
                     where x[nurse,day,work_period] == 1)(preference[nurse,day,work_period]);

% solve to maximize the number of preferred shifts
solve maximize sum_preference;

output
[
  "total: " ++ show(sum_preference) ++ "\n" ++
  "solution:\n\t    "
]
++
["Day \(j)\t" | j in 1..n_day]
++ 
[ 
  if j = 1 then "\n NURSE \(i): " else "\t" endif ++
    if fix(x[i,j,4]) == 1 then "--" elseif fix(x[i,j,3]) == 1  then "NUIT" elseif fix(x[i,j,2]) == 1  then "SOIREE" elseif fix(x[i,j,1]) == 1 then "MATIN" else "NONE" endif
  | i in 1..n_nurse, j in 1..n_day
];

