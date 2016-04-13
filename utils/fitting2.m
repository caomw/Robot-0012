function [ y_resampled ] = fitting2( y_data )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    index = 1;
    y_resampled = zeros(length(y_data),1);
    x_data = 0:1:length(y_data)-1;
    th = 5;
    %Getting segments | 3 -> poly2 requieres at least 3 point for fitting
    i=3;
    while (i<=length(y_data)-3)    
        if abs(y_data(i+1)-y_data(i)) > th
            index = horzcat(index,i,i+1);
            i=i+2;
        end
        i=i+1;
    end

    index = horzcat(index,length(y_data));

    %Fitting type
    f = fittype('poly2');
    %Fitting and Resampling
    for i = 1:2:length(index)-1
        fit1 = fit(x_data(index(i):index(i+1))',y_data(index(i):index(i+1)),f);
        y_resampled(index(i):index(i+1))= feval(fit1,x_data(index(i):index(i+1)));
    end
end

