%% read
fnames = dir('*cm.csv');
numfids = length(fnames);
vals = cell(1,numfids);
values=zeros(1,numfids);
data=values;
for K = 1:numfids
  files{K} = load(fnames(K).name);
  values(K)=str2num( strtok(fnames(K).name,'c'));
  data(K)=mean(files{K});
end


%% fit
p = polyfit(data,values,4);
%measured=linspace(values(1),values(end));
dist=polyval(p,data);

%% plot
plot(values,data,'.');
grid on
hold on
plot(values,values);
plot(values,dist,'.');