% Script to generate random experiment trials for subject numbers in the
% row vector 'subjects'.

function y=makesets(subjects)

% user impedance levels
K = [2,20,200];
B = [.25,2.5,25 250];
M = 1.5;

% moving object impedance levels
K2 = 20;
B2 = 2.5;
M2 = 1.5;

% impedance levels for training sets
K1t = 20000;
B1t = 20000;
M1t = 1.5;
K2t = 100;
B2t = 0;
M2t = 1.5;

nTrialsT = 10; % number of trials in feedback training sets
direction =  [-1, 1, 1, 1, -1, -1, 1, 1, -1, -1]; % direction of motion for training trials (same in all sets)

% feedback combinations to test
% [visual, vibrotactile, skin_stretch]
% in this experiment, we're only testing each type individually.
feedback = [1 0 0;
            0 1 0;];

% number of unique trials per set
nTrialsC = length(K)*length(B)*length(M);
% number of repetitions for experiment sets
nReps = 5;

% user impedance combinations for full factorial design
dff = fullfact([length(K) length(B) length(M)]);
for i=1:size(dff,1)
	imp(i,1) = K(dff(i,1));
	imp(i,2) = B(dff(i,2));
	imp(i,3) = M(dff(i,3));
end

for i=1:1:length(subjects)

	subjectNumber = subjects(i);

	fname = sprintf('Subject%02d.txt',subjectNumber);
	% check if set file(s) already exist(s) for this subject
	if (exist(fname,'file'))
		% prompt to overwrite
		mystr = sprintf('File already exists for subject %d. Overwrite (y/n)? ',subjectNumber);
		yn = input(mystr,'s');
		if (strcmp(yn,'y'))
			% then we will make sets for this subject
		else
			% skip this subject
			continue;
		end
  end
    
  output_file = fopen(fname,'w');
  
  % define feedback training sets (same for all subjects) by hand
  % order will be visual, vibrotactile, skin stretch
  for fb = 1:1:size(feedback,1)
    
    % first print set info:
    %   task type (always 1 for force minimization),
    %   number of trials,
    %   0 for constant impedance,
    %   2 for training set
    %   flags for feedback types
    fprintf(output_file,'1\n%d\n0\n2\n%d\n%d\n%d\n',nTrialsT,feedback(fb,1),feedback(fb,2),feedback(fb,3));

    for j=1:1:nTrialsT
      % print moving object impedance levels for this trial (same for entire set), in the order: k2, b2, m2
      fprintf(output_file,'%f\n%f\n%f\n',K2t,B2t,M2t);
      % print impedance levels for this trial (same for entire set), in the order: k, b, m
      fprintf(output_file,'%f\n%f\n%f\n',K1t,B1t,M1t);
      % direction multiplier
      fprintf(output_file,'%d\n',direction(j));
      fprintf(output_file,'\n');
    end
    
  end

  
  % randomize order of feedback combinations for the rest of the experiment
  fborder = randperm(size(feedback,1));
  
  % for each type of feedback...
  for fb = 1:1:size(feedback,1)

    % define constant impedance practice set (same for all subjects and feedback types) by hand
    % (no variable impedance practice in this experiment)
    practiceorder = [5, 2, 10, 6, 7, 4, 1, 3, 12, 8, 9, 11];

    % first print set info:
    %   task type (always 1 for force minimization),
    %   number of trials,
    %   0 for constant impedance,
    %   0 for practice set
    %   flags for feedback types
    fprintf(output_file,'1\n%d\n0\n0\n%d\n%d\n%d\n',length(practiceorder),feedback(fborder(fb),1),feedback(fborder(fb),2),feedback(fborder(fb),3));

    % define constant impedance practice set
    for j=1:1:nTrialsC
      % print moving object impedance levels for this trial (same for entire set), in the order: k2, b2, m2
      fprintf(output_file,'%f\n%f\n%f\n',K2,B2,M2);
      % print impedance levels for this trial, in the order: k, b, m
      fprintf(output_file,'%f\n%f\n%f\n',imp(practiceorder(j),1),imp(practiceorder(j),2),imp(practiceorder(j),3));
      % direction multiplier - randomly chosen +/-1
      fprintf(output_file,'%d\n',round(rand)*2-1);
      fprintf(output_file,'\n');
    end

    % define constant impedance experiment set

    % first print set info:
        %   task type (always 1 for force minimization),
        %   number of trials,
        %   0 for constant impedance,
        %   1 for practice set
        %   flags for feedback types
    fprintf(output_file,'1\n%d\n0\n1\n%d\n%d\n%d\n',nTrialsC*nReps,feedback(fborder(fb),1),feedback(fborder(fb),2),feedback(fborder(fb),3));

    for rep=1:1:nReps
      % define constant impedance trials
      trialorder = randperm(nTrialsC);
      for j=1:1:nTrialsC
        % print moving object impedance levels for this trial (same for entire set), in the order: k2, b2, m2
        fprintf(output_file,'%f\n%f\n%f\n',K2,B2,M2);
        % print impedance levels for this trial, in the order: k, b, m
        fprintf(output_file,'%f\n%f\n%f\n',imp(trialorder(j),1),imp(trialorder(j),2),imp(trialorder(j),3));
        % direction multiplier - randomly chosen +/-1
        fprintf(output_file,'%d\n',round(rand)*2-1);
        fprintf(output_file,'\n');
      end
    end
  end
  
  fclose(output_file);
end
