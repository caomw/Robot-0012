function [ pose, isPFLdone ] = PFL2( botScan, particles, isPFLdone, botEstimate )
%% create spaces to store information for the purpose of resampling
num = size(particles, 1);
newPos = zeros(num, 2);
newAng = zeros(num, 1);

%% Parameters setting
sigma = 3;
dampling = 0.01;

%% Updating weights
weights = ones(num, 1) * (1 / num); % initialize weights

for i = 1:num
    pScan = particles(i).ultraScan(); % get scan for each particle
    delta = sum( abs(pScan - botScan) );
    weights(i) = exp( - delta / (2 * sigma.^2) ) + dampling;
end

% normalize weights
addition = 0.5 / num;
weights = weights / sum(weights) + addition;

%% Resampling
count = 0;
totalNum = 0;
for i = 1:num
    % get max weight and the corresponding index of particle
    [w, index] = max(weights);

    % set current max weight to 0 for convenience of finding next max
    weights(index) = 0;

    % define number of particles that is going to be re-sampled
    offspringNum = round(w * num);

    % to avoid total number of particles exceeds pre-defined size
    if (totalNum + offspringNum) > num
        offspringNum = num - totalNum;
    end

    % copy particles
    for j = 1:offspringNum
        count = count + 1; % update count
        newPos(count, :) = particles(index).getBotPos();
        newAng(count) = particles(index).getBotAng();
    end

    % update total number of new-born particles
    totalNum = totalNum + offspringNum;
    if totalNum == num
        break
    end

end

%% get estimation
position = mean(newPos);
angle = meanangle(newAng);

%% update current particles
for i = 1:num
    particles(i).setBotPos( newPos(i, :) );
    particles(i).setBotAng( newAng(i) );
end

%% Write code to check for convergence
if isPFLdone == 0
    covmat = cov(newPos);
    eigval = eig(covmat);
    sumeig = sum(eigval) % threshold: 30(num=500)
end

%% return
pose = [position, angle];
if isPFLdone == 0 && sumeig < 80
    isPFLdone = 1;
end

%% check if the best estimate matches the real robot
threshold = 200;
if isPFLdone == 1
    botEstimate.setBotPos(position);
    botEstimate.setBotAng(angle);
    scan = botEstimate.ultraScan();
    delta = sum( abs(scan - botScan) )
    if delta > threshold % means the estimate is wrong
        isPFLdone = 0;
        % particles will be repositioned randomly
        for i = 1:num
            particles(i).randomPose(0);
        end
    end
end

end

