function [ y_resampled ] = fitting( y_data )
%UNTITLED Summary of this function goes here
%   Using a polynomial curve for getting floating data from integer data. 
%%Fitting
x_data = 0:1:length(y_data)-1; 
f = fittype('poly7'); 
fit1 = fit(x_data',y_data,f,'Robust','on','Normalize','on' );
%%Resampling
y_resampled = feval(fit1,x_data);
end

