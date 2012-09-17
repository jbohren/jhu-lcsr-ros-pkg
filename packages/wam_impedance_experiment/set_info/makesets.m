% Script to generate random experiment trials for subject numbers in the
% row vector 'subjects'.

function y=makesets(subjects)

% user impedance levels
K = [20 200 1000];
B = [0.1 1 10 40];

% user impedance combinations for full factorial design
dff = fullfact([length(K) length(B)]);
for i=1:size(dff,1)
	imp(i,1) = K(dff(i,1));
	imp(i,2) = B(dff(i,2));
end

% initial practice sets, k/b/direction
P1 = [K(3) B(4) 1; K(2) B(2) -1; K(1) B(1) 1; K(3) B(1) -1];
P2 = [K(2) B(2) 1; K(3) B(4) -1; K(1) B(1) 1; K(3) B(1) -1];

% full practice sets - one of each trial in this order
practiceorder = [5, 2, 10, 6, 7, 4, 1, 3, 12, 8, 9, 11];
practicedirections = [-1 1 1 1 -1 -1 1 -1 -1 1 -1 1];
P3 = [imp(practiceorder,:) practicedirections'];
P4 = [imp(practiceorder,:) practicedirections'];

% number of unique trials per set
nTrials = length(K)*length(B);
% number of experiment sets for each task
nSets = 2;
% number of repetitions for each experiment set
nReps = 2;
% length in seconds of each trial
trialLength = 15;

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

    % define initial practice sets
    
    % tracking practice set
    
    % first print set info:
        %   0 for tracking set,
        %   0 for practice set,
        %   number of trials,
        %   length of trials
    fprintf(output_file,'0\n0\n%d\n%f\n',size(P1,1),trialLength);
    
    for j=1:1:size(P1,1)
        % print k, b, direction on separate lines
        fprintf(output_file,'%f\n%f\n%d\n\n',P1(j,1),P1(j,2),P1(j,3));
    end
    
    % forcemin practice set
    
    % first print set info:
        %   1 for forcemin set,
        %   0 for practice set,
        %   number of trials,
        %   length of trials
    fprintf(output_file,'1\n0\n%d\n%f\n',size(P2,1),trialLength);
    
    for j=1:1:size(P2,1)
        % print k, b, direction on separate lines
        fprintf(output_file,'%f\n%f\n%d\n\n',P2(j,1),P2(j,2),P2(j,3));
    end
    
    % randomize task order
    
    firsttask = round(rand);
    
    if (firsttask == 0)
        
        % tracking task first
        
        % define practice set:
        %   0 for tracking task,
        %   0 for practice set,
        %   number of trials,
        %   length of trials
        fprintf(output_file,'0\n0\n%d\n%f\n',nTrials,trialLength);
    
        for j=1:1:size(P3,1)
            % print k, b, direction on separate lines
            fprintf(output_file,'%f\n%f\n%d\n\n',P3(j,1),P3(j,2),P3(j,3));
        end
    
        % define experiment sets
        for setNumber = 1:1:nSets
            %   0 for tracking task,
            %   1 for experiment set,
            %   number of trials,
            %   length of trials
            fprintf(output_file,'0\n1\n%d\n%f\n',nTrials*nReps,trialLength);

            for rep=1:1:nReps
                % define constant impedance trials
                trialorder = randperm(nTrials);
                for j=1:1:nTrials
                    % print impedance levels for this trial, in the order: k, b
                    fprintf(output_file,'%f\n%f\n',imp(trialorder(j),1),imp(trialorder(j),2));
                    % direction multiplier - randomly chosen +/-1
                    fprintf(output_file,'%d\n',round(rand)*2-1);
                    fprintf(output_file,'\n');
                end
            end
        end
    end
    
    % forcemin task
    
    % define practice set:
    %   1 for forcemin task,
    %   0 for practice set,
    %   number of trials,
    %   length of trials
    fprintf(output_file,'1\n0\n%d\n%f\n',nTrials,trialLength);

    for j=1:1:size(P3,1)
        % print k, b, direction on separate lines
        fprintf(output_file,'%f\n%f\n%d\n\n',P4(j,1),P4(j,2),P4(j,3));
    end

    % define experiment sets
    for setNumber = 1:1:nSets
        %   1 for forcemin task,
        %   1 for experiment set,
        %   number of trials,
        %   length of trials
        fprintf(output_file,'1\n1\n%d\n%f\n',nTrials*nReps,trialLength);

        for rep=1:1:nReps
            % define constant impedance trials
            trialorder = randperm(nTrials);
            for j=1:1:nTrials
                % print impedance levels for this trial, in the order: k, b
                fprintf(output_file,'%f\n%f\n',imp(trialorder(j),1),imp(trialorder(j),2));
                % direction multiplier - randomly chosen +/-1
                fprintf(output_file,'%d\n',round(rand)*2-1);
                fprintf(output_file,'\n');
            end
        end
    end
        
  
    if (firsttask == 1)
        
        % tracking task second
        
        % define practice set:
        %   0 for tracking task,
        %   0 for practice set,
        %   number of trials,
        %   length of trials
        fprintf(output_file,'0\n0\n%d\n%f\n',nTrials,trialLength);
    
        for j=1:1:size(P3,1)
            % print k, b, direction on separate lines
            fprintf(output_file,'%f\n%f\n%d\n\n',P3(j,1),P3(j,2),P3(j,3));
        end
    
        % define experiment sets
        for setNumber = 1:1:nSets
            %   0 for tracking task,
            %   1 for experiment set,
            %   number of trials,
            %   length of trials
            fprintf(output_file,'0\n1\n%d\n%f\n',nTrials*nReps,trialLength);

            for rep=1:1:nReps
                % define constant impedance trials
                trialorder = randperm(nTrials);
                for j=1:1:nTrials
                    % print impedance levels for this trial, in the order: k, b
                    fprintf(output_file,'%f\n%f\n',imp(trialorder(j),1),imp(trialorder(j),2));
                    % direction multiplier - randomly chosen +/-1
                    fprintf(output_file,'%d\n',round(rand)*2-1);
                    fprintf(output_file,'\n');
                end
            end
        end
    end
    
    fclose(output_file);
end
